import pickle
from trajectory import phase as Phs

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

Walk_phases = Phs.Phases(data_dict, 'jump')

for i in range(data_size * 3):
    Walk_phases.print_phase(i, True)
