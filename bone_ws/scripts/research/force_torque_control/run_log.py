"""Append-only per-run diagnostic log for mock/replay research."""
import json
import time
from rclpy.node import Node
from std_msgs.msg import String
from pus6_research_msgs.msg import TorqueCommand, AxialForceCommand, RawMotorStatus
from pus6_motor_driver.protocol import unpack_status_response

class RunLog(Node):
    def __init__(self,path):
        super().__init__('research_run_log')
        self.stream=open(path,'x',encoding='utf-8',buffering=1)
        self.counts={'imu':0,'control':0,'torque':0,'force':0,'status':0}
        for name,topic in [('imu','/research_imu_lengths/diagnostics'),('control','/research_force_torque_mock/diagnostics'),('status','/research_force_status/diagnostics')]:
            self.create_subscription(String,topic,lambda m,n=name:self.text_event(n,m),10)
        self.create_subscription(RawMotorStatus,'/research_force_status/raw_status',self.raw_status,10)
        self.create_subscription(TorqueCommand,'/research_force_torque_mock/torque_command',lambda m:self.command('torque',m.torque_nm,m.sample_monotonic_ns),10)
        self.create_subscription(AxialForceCommand,'/research_force_torque_mock/force_command',lambda m:self.command('force',m.force_n,m.sample_monotonic_ns),10)
    def record(self,kind,data):
        self.stream.write(json.dumps({'kind':kind,'host_monotonic_ns':time.monotonic_ns(),'data':data},allow_nan=False)+'\n')
    def text_event(self,name,msg):
        self.counts[name]+=1
        try: data=json.loads(msg.data)
        except (ValueError,TypeError): data=msg.data
        self.record(name,data)
    def raw_status(self,msg):
        try:
            status=unpack_status_response(bytes(msg.frame))
            self.record('raw_status',{'source':int(msg.source),'axis':int(msg.axis_id),'rx_monotonic_ns':int(msg.rx_monotonic_ns),
                'position_steps':status.current_position,'force_g_raw':status.force_g,'force_adc':status.force_adc,'fault':status.error_code})
        except Exception as exc: self.record('raw_status_error',str(exc))
    def command(self,name,values,sample_ns):
        self.counts[name]+=1
        self.record(name,{'values':[float(v) for v in values], 'unit':'N m' if name=='torque' else 'N','sample_monotonic_ns':int(sample_ns)})
    def close(self):
        self.stream.close()
