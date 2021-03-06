import pickle
from trajectory import interpolation as traj
from trajectory import phase as Phs
from curves import piecewise, polynomial
import numpy as np
import pandas as pd

with open('TSID_Trajectory.p', 'rb') as f:
    original_data =[]
    while True:
        try:
            data = pickle.load(f)
        except EOFError:
            break
        original_data.append(data)

data_dict = original_data[0]['TSID_Trajectories']
data_size = len(data_dict)

Walk_phases = Phs.Phases(data_dict)
CurveSet = traj.Interpolation(Walk_phases, 0.002)

cs = np.zeros((6, 11))
for i in range(len(Walk_phases.p)):
    cs[i, 0] = Walk_phases.getFinalTime(i)
    cs[i, 1:4] = Walk_phases.p[i].oMi_Rf.translation
    cs[i, 4:7] = Walk_phases.p[i].oMi_Lf.translation
    if Walk_phases.p[i].type == 0:
        cs[i, 7:10] = np.zeros((1, 3))
        cs[i, 10] = 0
    elif Walk_phases.p[i].type == 1:
        if Walk_phases.p[i].ssp == 'Lf': 
            cs[i, 7:10] = Walk_phases.p[i].oMf_Rf.translation
            cs[i, 10] = 1
        elif Walk_phases.p[i].ssp == 'Rf': 
            cs[i, 7:10] = Walk_phases.p[i].oMf_Lf.translation
            cs[i, 10] = -1
    else:
        cs[i, 7:10] = np.zeros((1, 3))
        cs[i, 10] = 2

import pandas as pd
df = pd.DataFrame(cs)
df.to_csv('cs.csv', index=False)

data = np.hstack( (CurveSet.com_traj, CurveSet.com_dot_traj) )
data_s = np.vstack(   (   np.array(CurveSet.time_traj), data.transpose()))

df = pd.DataFrame(data_s.T)
df.to_csv('com_traj.csv', index=False)

# contact phase : end_time, rfoot_pos, rfoot_ori, lfoot_pos, lfoot_ori, goal_pos, goal_ori, contact_type (=0, 1, 2)




# ''' for display '''

# import talos_conf as conf
# from tsid_biped import TsidBiped
# import vizutils
# import numpy as np
# import time

# tsid = TsidBiped(conf, conf.viewer)
# vizutils.addViewerSphere(tsid.viz, 'world/com', conf.SPHERE_RADIUS, conf.COM_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/com_ref', conf.SPHERE_RADIUS, conf.COM_REF_SPHERE_COLOR)

# current_time = 0.0
# j = 0
# for i in range(0, len(CurveSet.time_traj)):
#     if (CurveSet.time_traj[i] > CurveSet.time_ori[j]):
#         j += 1
        
#     vizutils.applyViewerConfiguration(tsid.viz, 'world/com', CurveSet.com_ori[:, j].tolist() + [0, 0, 0, 1.])
#     vizutils.applyViewerConfiguration(tsid.viz, 'world/com_ref', CurveSet.com_traj[i].tolist() + [0, 0, 0, 1.])
#     time.sleep( (CurveSet.time_traj[i+1]-CurveSet.time_traj[i])  / 2.0)

