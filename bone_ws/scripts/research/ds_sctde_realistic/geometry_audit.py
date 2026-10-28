"""Finite local nominal-model geometry audit; not workspace qualification."""
import csv
import copy
import itertools
import sys
from pathlib import Path
import numpy as np
from realistic_simulation import KIN, nonlinear_geometry
from kinematics_map import attitude_jacobian


def audit():
    rows=[]
    for eta in itertools.product([-.1,0.,.1],repeat=3):
        eta=np.array(eta); q,j=nonlinear_geometry(eta)
        pose=[0,0,KIN.nominal_height,*eta]; jf=attitude_jacobian(KIN,pose)
        initial=np.asarray(pose)+np.array([.0001,-.0001,.0001,.001,-.001,.001])
        fk=KIN.forward_kinematics_with_metrics(q,initial_guess=initial)
        force=np.array([1.,-2.,3.,-1.,2.,-3.]); vel=np.array([.1,-.2,.15])
        rows.append({'roll_rad':eta[0],'pitch_rad':eta[1],'yaw_rad':eta[2],
             'condition':float(np.linalg.cond(j)),'q_min_m':float(min(q)),'q_max_m':float(max(q)),
             'jacobian_max_difference_m_per_rad':float(np.max(np.abs(j-jf))),
             'virtual_power_residual':float(abs(force@(j@vel)-(j.T@force)@vel)),
             'fk_converged':int(fk.converged),'fk_residual_m':fk.residual_norm,
             'fk_pose_error_norm':float(np.linalg.norm(np.asarray(fk.pose)-np.asarray(pose)))})
    return rows


def sensitivity():
    """Illustrative +/-2% dimensions; not inferred uncertainty bounds."""
    nominal_pose=np.array([.05,-.04,.03]); q0,j0=nonlinear_geometry(nominal_pose)
    rows=[]
    for parameter in ('base_anchors','platform_anchors_body','strut_length'):
        for fraction in (-.02,.02):
            kin=copy.deepcopy(KIN)
            if parameter=='strut_length': kin.strut_length *= 1+fraction
            else:
                value=np.asarray(getattr(kin,parameter),dtype=float)*(1+fraction)
                setattr(kin,parameter,value.tolist())
            q,j=nonlinear_geometry(nominal_pose,kin)
            rows.append({'parameter':parameter,'relative_change':fraction,
                'max_abs_delta_q_m':float(np.max(np.abs(q-q0))),
                'max_abs_delta_j_m_per_rad':float(np.max(np.abs(j-j0))),
                'condition':float(np.linalg.cond(j))})
    return rows


if __name__=='__main__':
    directory=Path(sys.argv[1])
    for name,rows in [('geometry_workspace.csv',audit()),('geometry_sensitivity.csv',sensitivity())]:
        output=directory/name
        if output.exists(): raise SystemExit('refusing to overwrite')
        with output.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
