"""Dual-IMU stabilization using the existing timed-feedback/Mode-3 command path."""
import numpy as np
from dataclasses import replace
from scipy.spatial.transform import Rotation
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from .force_control_node import TaskSpaceForceControlNode
from .active_stabilization import ActiveStabilizer, StabilizationConfig, ImuHistory
from .joint_position_force_pid import JointPositionForcePID, JointForcePIDConfig
from .force_pipeline import raw_steps_to_slider_m
from pus6_kinematics.state_synchronizer import SyncStatus


class ForceActiveStabilizationNode(TaskSpaceForceControlNode):
    def __init__(self, parameter_overrides=None, *, clock=None):
        super().__init__('force_active_stabilization',parameter_overrides=parameter_overrides,clock=clock)
        self.timer.cancel()
        if not self.use_fixed_lag or self.fixed_lag_delay_ns<=0:
            raise ValueError('active stabilization uses the existing positive-delay Fixed-Lag chain')
        # No external-force target or Z PID participates in this controller.
        self.target_timeout_s=0.
        self.destroy_subscription(self.target_sub)
        defaults=StabilizationConfig();values={}
        for name,value in vars(defaults).items():
            self.declare_parameter(name,list(value) if isinstance(value,tuple) else value)
            values[name]=self.get_parameter(name).value
        self.stabilizer=ActiveStabilizer(StabilizationConfig(**values),clock=self.clock)
        self.declare_parameter('joint_execution_only',False)
        self.joint_execution_only=bool(self.get_parameter('joint_execution_only').value)
        self.imu_target_suspended=False;self.imu_suspension_reason=None
        self.declare_parameter('execution_backend','task_wrench')
        self.execution_backend=self.get_parameter('execution_backend').value
        if self.execution_backend not in ('task_wrench','joint_position_force'):
            raise ValueError('unknown execution backend')
        joint_values={}
        for name,value in vars(JointForcePIDConfig()).items():
            self.declare_parameter(name,list(value) if isinstance(value,tuple) else value)
            joint_values[name]=self.get_parameter(name).value
        geom=self.kinematics.geometry_config
        geometry_min=raw_steps_to_slider_m([geom.normal_motion_min_steps]*6,geom)
        geometry_max=raw_steps_to_slider_m([geom.normal_motion_max_steps]*6,geom)
        joint_values['joint_min_position_m']=np.maximum(joint_values['joint_min_position_m'],geometry_min).tolist()
        joint_values['joint_max_position_m']=np.minimum(joint_values['joint_max_position_m'],geometry_max).tolist()
        self.joint_pid=JointPositionForcePID(JointForcePIDConfig(**joint_values),clock=self.clock)
        if self.joint_pid.config.joint_active_braking:
            self.drive.limits=replace(self.drive.limits,brake_rate_g_s=self.joint_pid.config.joint_brake_rate_g_s)
        self.joint_reference_override_m=None  # bounded execution experiment, never a position UART command
        self.joint_capture_m=None;self.ik_capture_m=None
        self.declare_parameter('imu_timeout_s',.1)
        self.declare_parameter('max_displacement_steps',6)
        if not self.has_parameter('max_current_ma'):
            self.declare_parameter('max_current_ma', 500)
        self.declare_parameter('allow_mock_hardware', False)
        self.allow_mock_hardware = bool(self.get_parameter('allow_mock_hardware').value)
        self.imu_timeout_ns=int(self.get_parameter('imu_timeout_s').value*1e9)
        self.displacement_steps=int(self.get_parameter('max_displacement_steps').value)
        self.max_current_ma=int(self.get_parameter('max_current_ma').value)
        if self.imu_timeout_ns<=0 or self.displacement_steps<0 or (self.displacement_steps==0 and self.execution_backend!='joint_position_force'):
            raise ValueError('positive IMU age/legacy travel guard required; joint backend always enforces absolute and tracking bounds')
        self.base_imu=ImuHistory();self.platform_imu=ImuHistory()
        self.declare_parameter('auto_enable', False)
        auto_en = bool(self.get_parameter('auto_enable').value)
        self.enabled = auto_en
        self.capture_pending = auto_en
        self.hold_reference_ns=None
        self.initial_positions_m=None;self.stabilization_diagnostics={}
        self._last_wait_log_ns = 0
        self._last_wait_fb_ns = 0
        self._last_wait_imu_ns = 0
        self._last_status_log_ns = 0
        if auto_en:
            self.get_logger().info('auto_enable=True: Active stabilization automatically enabled on startup.')
        self.base_sub=self.create_subscription(Imu,'/pus6/imu_base/data',lambda m:self._imu(m,self.base_imu),qos_profile_sensor_data)
        self.platform_sub=self.create_subscription(Imu,'/pus6/imu_platform/data',lambda m:self._imu(m,self.platform_imu),qos_profile_sensor_data)
        self.enable_service=self.create_service(SetBool,'~/enable',self._enable)
        self.timer=self.create_timer(self.period_s,self._control_cycle)

    def _imu(self,msg,history):
        try:
            q=([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
                if msg.orientation_covariance[0]>=0 else None)
            gyro=([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
                if msg.angular_velocity_covariance[0]>=0 else None)
            history.push(self.clock(),q,gyro)
        except Exception as exc:
            self._trip_fault('INVALID_IMU',str(exc),self.clock())

    def _enable(self,request,response):
        if request.data and self.fault_latched:
            response.success=False;response.message='reset fault first; no motor mode change';return response
        self.enabled=bool(request.data)
        self.capture_pending=self.enabled
        if self.enabled:
            if self.imu_target_suspended:self.joint_reference_override_m=None
            self.imu_target_suspended=False;self.imu_suspension_reason=None
        if not self.enabled:
            self.stabilizer.reset()
            self.joint_pid.initialized=False
            if hasattr(self.joint_pid,'integral'):self.joint_pid.integral[:]=0.
            self.stabilization_diagnostics={}
        if not self.enabled:
            self.request_terminal_stop('operator_pause')
        response.success=True
        response.message='Reference capture requested' if self.enabled else 'Owner terminal handover requested; await Mode0/anchor confirmation'
        return response

    def _reset_control_state(self,now_ns):
        super()._reset_control_state(now_ns)
        if hasattr(self,'enabled'):self.enabled=False
        if hasattr(self,'stabilizer'):self.stabilizer.reset()
        if hasattr(self,'joint_pid'):
            self.joint_pid.initialized=False
            if hasattr(self.joint_pid,'integral'):self.joint_pid.integral[:]=0.

    def _trip_fault(self,status,reason,now_ns):
        # Missing attitude interpolation freezes only the validated position
        # reference. Actual actuator/control failures terminate through the owner.
        if self.fault_latched:return
        if (status in ('SYNC_LOST','INVALID_SYNC_FORCE') and self.execution_backend=='joint_position_force'
                and self.joint_pid.config.joint_use_latest_feedback and self.joint_pid.initialized):
            self._suspend_attitude_target(reason);return
        super()._trip_fault(status,reason,now_ns)

    def _control_cycle(self):
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            return
        now=self.clock()
        if not getattr(self,'enabled',False):
            if now - getattr(self, '_last_wait_log_ns', 0) > 3_000_000_000:
                self._last_wait_log_ns = now
                self.get_logger().info('[WAITING] Node initialized (dry_run=True). Awaiting enable via service ~/enable or parameter auto_enable:=true')
            return
        if self.fault_latched:
            if now - getattr(self, '_last_wait_log_ns', 0) > 3_000_000_000:
                self._last_wait_log_ns = now
                self.get_logger().warn(f'[FAULT LATCHED] Status: {self.fault_reason}. Reset via service ~/reset_fault')
            return
        fb=self.latest_timed_feedback
        if fb is None:
            if now - getattr(self, '_last_wait_fb_ns', 0) > 2_000_000_000:
                self._last_wait_fb_ns = now
                base_cnt = len(self.base_imu.samples)
                plat_cnt = len(self.platform_imu.samples)
                self.get_logger().info(f'[WAITING FEEDBACK] Awaiting /pus6/joint_feedback/timed (Base IMU: {base_cnt} samples, Plat IMU: {plat_cnt} samples)')
            return
        if (not fb.force_mode_confirmed or not fb.force_hold_valid
                or not 0<fb.force_hold_reference_monotonic_ns<=now):
            self._trip_fault('INVALID_HANDOVER','no acknowledged Mode 3 holding reference',now);return
        if not self.dry_run and fb.source!=fb.SOURCE_HARDWARE and not self.allow_mock_hardware:
            self._trip_fault('NON_HARDWARE_SOURCE','live commands require hardware feedback',now);return
        if self.hold_reference_ns is None:
            try:
                self.initialize_hold_forces_g(list(fb.force_hold_g),now_ns=now)
                self.hold_reference_ns=fb.force_hold_reference_monotonic_ns
                self.initial_positions_m=np.array(fb.positions_meters)
            except Exception as exc:
                self._trip_fault('INVALID_HOLD',str(exc),now);return
        elif (self.hold_reference_ns!=fb.force_hold_reference_monotonic_ns
                or tuple(fb.force_hold_g)!=self.pipeline.hold_forces_g):
            self._trip_fault('HANDOVER_CHANGED','holding reference changed; restart explicitly',now);return
        if hasattr(fb, 'currents_ma') and len(fb.currents_ma) == 6:
            if max(abs(c) for c in fb.currents_ma) > self.max_current_ma:
                self._trip_fault('CURRENT_LIMIT', f'configured motor current protection exceeded ({max(abs(c) for c in fb.currents_ma)} > {self.max_current_ma} mA)', now)
                return
        for history, name in ((self.base_imu, 'Base IMU'), (self.platform_imu, 'Platform IMU')):
            if self.joint_execution_only or self.imu_target_suspended:
                continue
            if history.latest_complete_ns<=0 or now-history.latest_complete_ns>self.imu_timeout_ns:
                if self.stabilizer.reference is None:
                    if now - getattr(self, '_last_wait_imu_ns', 0) > 2_000_000_000:
                        self._last_wait_imu_ns = now
                        self.get_logger().info(f'[WAITING IMU] Waiting for fresh {name} samples (complete_ns={history.latest_complete_ns})')
                    return  # normal IMU startup bracketing, no fake reference
                if self.execution_backend=='joint_position_force' and self.joint_pid.config.joint_use_latest_feedback:
                    self._suspend_attitude_target('IMU missing/stale; position loop retained')
                    break
                if self.stabilizer.reference is None:return
                self._trip_fault('IMU_TIMEOUT',f'{name} feedback missing/stale',now);return
        if self.execution_backend=='joint_position_force':
            try:self.joint_pid.validate_position(fb.positions_meters)
            except ValueError as exc:
                self._trip_fault('ABSOLUTE_TRAVEL',str(exc),now);return
        if self.initial_positions_m is not None and self.displacement_steps>0:
            limit=self.displacement_steps*self.kinematics.geometry_config.meters_per_step
            if np.max(np.abs(np.asarray(fb.positions_meters)-self.initial_positions_m))>limit+1e-12:
                self._trip_fault('DISPLACEMENT','configured experiment displacement guard exceeded',now);return
        self.last_target_update_ns=now  # no external force-reference stream
        # Both IMUs must bracket the SAME existing Fixed-Lag motor query time.
        if self.joint_execution_only or self.imu_target_suspended:
            self._position_execution_cycle(fb,now);return
        query=self._sync_query_stamp(now,fb)
        if query<self.hold_reference_ns:return
        if not self.joint_execution_only and not self.imu_target_suspended:
            try:
                self.aligned_base=self.base_imu.query(query,self.imu_timeout_ns)
                self.aligned_platform=self.platform_imu.query(query,self.imu_timeout_ns)
            except ValueError as exc:
                if self.stabilizer.reference is None:return
                if self.execution_backend=='joint_position_force' and self.joint_pid.config.joint_use_latest_feedback:
                    self._suspend_attitude_target(str(exc))
                else:
                    if self.stabilizer.reference is not None:self._trip_fault('IMU_ALIGNMENT',str(exc),now)
                    return
        super()._control_cycle()

    def _sync_query_stamp(self,now_ns,fb):
        base_stamp = getattr(fb, 'snapshot_end_monotonic_ns', 0) or now_ns
        return int(base_stamp) - self.fixed_lag_delay_ns

    def _position_execution_cycle(self,fb,now):
        # Position execution needs fresh individual measurements, NOT an IMU or
        # a new task-wrench estimate. Missing alignment suppresses only wrench
        # publication; never reconstruct a wrench from mixed/stale times.
        error=self._feedback_error(fb,now)
        if error or now-self.last_feedback_recv_ns>self.feedback_timeout_s*1e9:
            self._trip_fault('FEEDBACK_TIMEOUT',error or 'position feedback timeout',now);return
        if fb.uart_cycle_sequence==self.last_processed_cycle_seq:return
        if self.drive.last_command_time_ns is not None and now<=self.drive.last_command_time_ns:return
        query=self._sync_query_stamp(now,fb);sync=self.synchronizer.query_sync_configuration(query)
        pose=None;aligned_positions=None;aligned=False
        forces=list(fb.forces_g_raw)
        if (sync.status==SyncStatus.SYNCED and all(sync.force_valid) and not any(sync.fault_codes)
                and all(r-l<=self.max_feedback_gap_s*1e9 and now-l<=self.feedback_timeout_s*1e9 for l,r in zip(sync.per_axis_left_stamps_ns,sync.per_axis_right_stamps_ns))):
            try:
                fk=self.kinematics.forward_kinematics_with_metrics(sync.positions_meters,initial_guess=self.current_pose_estimate)
                if fk.converged and fk.residual_norm<=self.max_fk_residual_m:
                    pose=np.asarray(fk.pose);self.current_pose_estimate=list(fk.pose)
                    forces=list(sync.forces_g_raw);aligned_positions=list(sync.positions_meters);aligned=True
            except (ValueError,RuntimeError):pass # optional monitor; joint target stays frozen
        try:cycle=self._compute_joint_force_cycle(forces,pose,now,query,None,None,None,None)
        except Exception as exc:
            self._trip_fault('POSITION_EXECUTION_FAILURE',str(exc),now);return
        self.stabilization_diagnostics['wrench_monitor_aligned']=aligned
        self._publish_control_cycle(cycle,fb,now,query if aligned else None,forces,aligned_positions,
            source_stamps_ns=([query]*6 if aligned else fb.per_axis_rx_done_monotonic_ns))

    def _suspend_attitude_target(self,reason):
        if not self.imu_target_suspended:
            self.imu_target_suspended=True;self.imu_suspension_reason=reason
            if self.joint_pid.initialized:
                self.joint_reference_override_m=self.joint_pid.reference.value.copy()
            self.get_logger().warn('Attitude target suspended; Mode3 position servo continues: '+reason)

    def _compute_force_cycle(self,raw_forces_g,pose,now_ns,measurement_ns):
        if self.execution_backend=='joint_position_force' and (self.joint_execution_only or self.imu_target_suspended):
            return self._compute_joint_force_cycle(raw_forces_g,pose,now_ns,measurement_ns,None,None,None,None)
        try:
            bq,bg=self.base_imu.query(measurement_ns,self.imu_timeout_ns)
            pq,pg=self.platform_imu.query(measurement_ns,self.imu_timeout_ns)
        except ValueError as exc:
            if self.execution_backend=='joint_position_force' and self.joint_pid.config.joint_use_latest_feedback:
                self._suspend_attitude_target(str(exc))
                return self._compute_joint_force_cycle(raw_forces_g,pose,now_ns,measurement_ns,None,None,None,None)
            raise
        if self.execution_backend=='joint_position_force':
            return self._compute_joint_force_cycle(raw_forces_g,pose,now_ns,measurement_ns,bq,pq,bg,pg)
        if self.capture_pending:
            self.stabilizer.capture(pose,bq,pq,measurement_ns)
            self.capture_pending=False
            wrench=np.zeros(6)
            self.stabilization_diagnostics={}
            self.get_logger().info(
                f"[STABILIZATION CAPTURE] Reference captured: "
                f"FK_XYZ_mm={np.round(np.asarray(pose[:3])*1000, 2).tolist()}, "
                f"FK_RPY_deg={np.round(np.rad2deg(pose[3:6]), 2).tolist()}"
            )
        else:
            wrench,self.stabilization_diagnostics=self.stabilizer.update(pose,bq,pq,bg,pg,
                sample_ns=measurement_ns,now_ns=now_ns)
        cycle=self.pipeline.command_increment_wrench(wrench,raw_forces_g,pose,now_ns=now_ns)
        self.stabilizer.track_actuator_output(cycle.applied_wrench,cycle.actuator_limited)
        self.stabilization_diagnostics.update(controller_mode='POSITION_M2_REFERENCE_TO_POSE_WRENCH_PID',
            measured_wrench_n_nm=cycle.measured_wrench.tolist(),
            applied_wrench_prediction_n_nm=cycle.applied_wrench.tolist(),
            wrench_response_error_n_nm=(wrench-cycle.measured_wrench).tolist())
        if now_ns - getattr(self, '_last_status_log_ns', 0) > 1_000_000_000:
            self._last_status_log_ns = now_ns
            try:
                base_rpy = np.round(self.stabilization_diagnostics.get('base_rel_rpy_deg', [0, 0, 0]), 2).tolist()
                plat_rpy = np.round(self.stabilization_diagnostics.get('platform_rel_rpy_deg', [0, 0, 0]), 2).tolist()
                err_rpy_deg = np.round(np.rad2deg(self.stabilization_diagnostics.get('platform_error_world_rad', [0, 0, 0])), 3).tolist()
                cmd_w = np.round(np.asarray(wrench), 4).tolist()
                disp_g = np.round(np.asarray(cycle.drive_result['dispatched_forces_g']), 1).tolist()
                delta_g = np.round(np.asarray(cycle.drive_result['dispatched_forces_g']) - np.asarray(self.pipeline.hold_forces_g), 1).tolist()
                self.get_logger().info(
                    f"[STATUS] BaseRelRPY(deg): {base_rpy} | PlatRelRPY(deg): {plat_rpy} | "
                    f"ErrRPY(deg): {err_rpy_deg} | "
                    f"PID Wrench(N,Nm): {cmd_w} | "
                    f"dF(g): {delta_g} | TargetForce(g): {disp_g}"
                )
            except Exception as log_exc:
                self.get_logger().debug(f"Status logging exception: {log_exc}")
        return cycle

    def _compute_joint_force_cycle(self,raw_forces_g,pose,now_ns,measurement_ns,bq,pq,bg,pg):
        # SAME Fixed-Lag time for q, force, FK and dual IMUs.
        measured=None;position_sample_ns=measurement_ns;stamps=None
        if not self.joint_pid.config.joint_use_latest_feedback:
            measured=np.asarray(self.synchronizer.query_sync_configuration(measurement_ns).positions_meters)
        if self.joint_pid.config.joint_use_latest_feedback:
            fb=self.latest_timed_feedback
            measured=np.asarray(fb.positions_meters);stamps=list(fb.per_axis_rx_done_monotonic_ns)
            position_sample_ns=fb.snapshot_end_monotonic_ns
        if self.capture_pending:
            if not (self.joint_execution_only or self.imu_target_suspended):self.stabilizer.capture(pose,bq,pq,measurement_ns)
            self.joint_capture_m=measured.copy()
            self.ik_capture_m=(None if pose is None else np.asarray(self.kinematics.inverse_kinematics(pose)))
            self.joint_pid.reset(measured,sample_ns=position_sample_ns,now_ns=now_ns,position_stamps_ns=stamps)
            self.capture_pending=False;delta=np.zeros(6)
            self.stabilization_diagnostics=dict(joint_target_m=measured.tolist(),joint_reference_m=measured.tolist(),
                joint_measured_m=measured.tolist(),joint_error_m=[0.]*6,joint_delta_force_n=[0.]*6)
        else:
            if self.joint_execution_only or self.imu_target_suspended:
                target=self.joint_capture_m.copy();attitude={}
            else:
                desired,attitude=self.stabilizer.target_pose(pose,bq,pq,bg,pg,sample_ns=measurement_ns)
            # Position's differential IK, anchored at measured q rather than a
            # nominal HOME. This removes the captured FK numerical residual.
                target=self.joint_capture_m+np.asarray(self.kinematics.inverse_kinematics(desired))-self.ik_capture_m
            if self.joint_reference_override_m is not None:
                target=np.asarray(self.joint_reference_override_m)
            delta,joint=self.joint_pid.update(target,measured,sample_ns=position_sample_ns,now_ns=now_ns,position_stamps_ns=stamps)
            self.stabilization_diagnostics=dict(attitude,**joint)
        cycle=self.pipeline.command_actuator_increment(delta,raw_forces_g,pose,now_ns=now_ns,braking_axes=self.joint_pid.braking_axes)
        applied_n=self.pipeline.compensation.command_n(cycle.drive_result['dispatched_forces_g'])-self.pipeline.hold_forces_n
        self.joint_pid.track_applied(applied_n)
        self.stabilization_diagnostics.update(controller_mode=('REAL_POSITION_FORCE_EXECUTION' if self.joint_execution_only else 'IMU_TARGET_FROZEN_POSITION_FORCE_HOLD' if self.imu_target_suspended else 'POSITION_M2_DIFFERENTIAL_IK_TO_JOINT_POSITION_FORCE_PID'),
            joint_execution_only=self.joint_execution_only,attitude_target_suspended=self.imu_target_suspended,
            attitude_suspension_reason=self.imu_suspension_reason,position_source_sample_ns=position_sample_ns,
            hold_force_n=self.pipeline.hold_forces_n.tolist(),applied_delta_force_n=applied_n.tolist(),
            measured_delta_force_n=self.pipeline.relative_feedback_n(raw_forces_g).tolist(),
            measured_wrench_n_nm=None if cycle.measured_wrench is None else cycle.measured_wrench.tolist(),
            applied_wrench_prediction_n_nm=None if cycle.applied_wrench is None else cycle.applied_wrench.tolist())
        if now_ns - getattr(self, '_last_status_log_ns', 0) > 1_000_000_000:
            self._last_status_log_ns = now_ns
            try:
                disp_g = np.round(np.asarray(cycle.drive_result['dispatched_forces_g']), 1).tolist()
                delta_g = np.round(np.asarray(cycle.drive_result['dispatched_forces_g']) - np.asarray(self.pipeline.hold_forces_g), 1).tolist()
                fk_str = f"FK(mm,deg): {np.round(np.asarray(pose[:3])*1000, 2).tolist()}, {np.round(np.rad2deg(pose[3:6]), 2).tolist()}" if pose is not None else "FK: None"
                self.get_logger().info(
                    f"[STATUS JOINT] {fk_str} | "
                    f"TargetForce(g): {disp_g} (dF: {delta_g})"
                )
            except Exception:
                pass
        return cycle

    def _emit_diagnostics(self,status,details,now_ns):
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            return
        details.update(getattr(self,'stabilization_diagnostics',{}))
        try:
            super()._emit_diagnostics(status,details,now_ns)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node=ForceActiveStabilizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            if rclpy.ok() and hasattr(node, 'context') and node.context.ok():
                node.request_terminal_stop('stabilization_process_exit')
        except Exception:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
