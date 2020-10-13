import pickle
from trajectory import phase as Phs

with open('walk_terrain_0.p', 'rb') as f:
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

# for i in range(data_size * 3):
#     Walk_phases.print_phase(i, True)

print (data_dict)