import pickle
from trajectory import interpolation as traj
from trajectory import phase as Phs
from curves import piecewise, polynomial
import numpy as np
import pandas as pd

with open('TSID_jump_Trajectory.p', 'rb') as f:
    original_data =[]
    while True:
        try:
            data = pickle.load(f)
        except EOFError:
            break
        original_data.append(data)

data_dict = original_data[0]['TSID_Trajectories']
data_size = len(data_dict)

print (data_dict)
Walk_phases = Phs.Phases(data_dict, 'jump')
CurveSet = traj.Interpolation(Walk_phases, 0.002)

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

