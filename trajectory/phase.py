import pinocchio as pin
import numpy as np
np.set_printoptions(precision=4)
class Phase:
    def __init__(self, dict, index, type, scenario = 'walk'):
        self.type = type
        self.scenario = scenario

        if type == 0: # init dsp
            if index == 0:
                self.time = dict[index]['InitDouble_TimeSeries']
                self.com_x = dict[index]['InitDouble_x']
                self.com_y = dict[index]['InitDouble_y']
                self.com_z = dict[index]['InitDouble_z']
                self.com_x_dot = dict[index]['InitDouble_xdot']
                self.com_y_dot = dict[index]['InitDouble_ydot']
                self.com_z_dot = dict[index]['InitDouble_zdot']


                self.Lx = dict[index]['InitDouble_Lx']
                self.Ly = dict[index]['InitDouble_Ly']
                self.Lz = dict[index]['InitDouble_Lz']
                self.Lx_dot = dict[index]['InitDouble_Ldotx']
                self.Ly_dot = dict[index]['InitDouble_Ldoty']
                self.Lz_dot = dict[index]['InitDouble_Ldotz']

            else: 
                time_offset = 0.0
                for i in range(1, index+1):
                    time_offset += dict[i-1]['DoubleSupport_TimeSeries'][-1]

                self.time = dict[index]['InitDouble_TimeSeries'][1:] + time_offset
                self.com_x = dict[index]['InitDouble_x'][1:]
                self.com_y = dict[index]['InitDouble_y'][1:] 
                self.com_z = dict[index]['InitDouble_z'][1:] 
                self.com_x_dot = dict[index]['InitDouble_xdot'][1:] 
                self.com_y_dot = dict[index]['InitDouble_ydot'][1:] 
                self.com_z_dot = dict[index]['InitDouble_zdot'][1:] 
                self.Lx = dict[index]['InitDouble_Lx'][1:] 
                self.Ly = dict[index]['InitDouble_Ly'][1:] 
                self.Lz = dict[index]['InitDouble_Lz'][1:]
                self.Lx_dot = dict[index]['InitDouble_Ldotx'][1:] 
                self.Ly_dot = dict[index]['InitDouble_Ldoty'][1:] 
                self.Lz_dot = dict[index]['InitDouble_Ldotz'][1:] 

        elif type == 1: # ssp
            if index == 0:
                self.time = dict[index]['Swing_TimeSeries'][1:]
                self.com_x = dict[index]['Swing_x'][1:] 
                self.com_y = dict[index]['Swing_y'][1:]
                self.com_z = dict[index]['Swing_z'][1:]
                self.com_x_dot = dict[index]['Swing_xdot'][1:] 
                self.com_y_dot = dict[index]['Swing_ydot'][1:] 
                self.com_z_dot = dict[index]['Swing_zdot'][1:] 
                self.Lx = dict[index]['Swing_Lx'][1:]
                self.Ly = dict[index]['Swing_Ly'][1:]
                self.Lz = dict[index]['Swing_Lz'][1:]
                self.Lx_dot = dict[index]['Swing_Ldotx'][1:] 
                self.Ly_dot = dict[index]['Swing_Ldoty'][1:] 
                self.Lz_dot = dict[index]['Swing_Ldotz'][1:] 

            else: 
                time_offset = 0
                for i in range(1, index+1):
                    time_offset += dict[i-1]['DoubleSupport_TimeSeries'][-1]
                self.time = dict[index]['Swing_TimeSeries'][1:]+ time_offset     
                self.com_x = dict[index]['Swing_x'][1:]
                self.com_y = dict[index]['Swing_y'][1:] 
                self.com_z = dict[index]['Swing_z'][1:] 
                self.com_x_dot = dict[index]['Swing_xdot'][1:] 
                self.com_y_dot = dict[index]['Swing_ydot'][1:] 
                self.com_z_dot = dict[index]['Swing_zdot'][1:] 

                self.Lx = dict[index]['Swing_Lx'][1:]
                self.Ly = dict[index]['Swing_Ly'][1:] 
                self.Lz = dict[index]['Swing_Lz'][1:] 
                self.Lx_dot = dict[index]['Swing_Ldotx'][1:] 
                self.Ly_dot = dict[index]['Swing_Ldoty'][1:] 
                self.Lz_dot = dict[index]['Swing_Ldotz'][1:]    
               
        elif type == 2: # dsp
            if index == 0:
                self.time = dict[index]['DoubleSupport_TimeSeries'][1:]
                self.com_x = dict[index]['DoubleSupport_x'][1:] 
                self.com_y = dict[index]['DoubleSupport_y'][1:]
                self.com_z = dict[index]['DoubleSupport_z'][1:]
                self.com_x_dot = dict[index]['DoubleSupport_xdot'][1:] 
                self.com_y_dot = dict[index]['DoubleSupport_ydot'][1:] 
                self.com_z_dot = dict[index]['DoubleSupport_zdot'][1:] 

                self.Lx = dict[index]['DoubleSupport_Lx'][1:]
                self.Ly = dict[index]['DoubleSupport_Ly'][1:]
                self.Lz = dict[index]['DoubleSupport_Lz'][1:]
                self.Lx_dot = dict[index]['DoubleSupport_Ldotx'][1:] 
                self.Ly_dot = dict[index]['DoubleSupport_Ldoty'][1:] 
                self.Lz_dot = dict[index]['DoubleSupport_Ldotz'][1:]    

            else: 
                time_offset = 0
                for i in range(1, index+1):
                    time_offset += dict[i-1]['DoubleSupport_TimeSeries'][-1]
                self.time = dict[index]['DoubleSupport_TimeSeries'][1:] + time_offset  
                self.com_x = dict[index]['DoubleSupport_x'][1:] 
                self.com_y = dict[index]['DoubleSupport_y'][1:] 
                self.com_z = dict[index]['DoubleSupport_z'][1:]
                self.com_x_dot = dict[index]['DoubleSupport_xdot'][1:] 
                self.com_y_dot = dict[index]['DoubleSupport_ydot'][1:] 
                self.com_z_dot = dict[index]['DoubleSupport_zdot'][1:] 

                self.Lx = dict[index]['DoubleSupport_Lx'][1:] 
                self.Ly = dict[index]['DoubleSupport_Ly'][1:] 
                self.Lz = dict[index]['DoubleSupport_Lz'][1:] 
                self.Lx_dot = dict[index]['DoubleSupport_Ldotx'][1:] 
                self.Ly_dot = dict[index]['DoubleSupport_Ldoty'][1:] 
                self.Lz_dot = dict[index]['DoubleSupport_Ldotz'][1:]           

        quat = np.array(dict[index]['Init_L_quat']).tolist()
        self.oMi_Lf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose()) , np.array(dict[index]['Init_PL']))
        quat = np.array(dict[index]['Init_R_quat']).tolist()
        self.oMi_Rf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose() ) , np.array(dict[index]['Init_PR']))
  
        self.oMf_Rf = []
        self.oMf_Lf = []

        if self.scenario == 'walk':
            if dict[index]['LeftSwingFlag'] == 0:
                quat = np.array(dict[index]['Landing_quat']).tolist()
                self.oMf_Rf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose() ), np.array(dict[index]['Landing_P']))
                self.ssp = 'Lf'
            elif dict[index]['RightSwingFlag'] == 0:
                quat = np.array(dict[index]['Landing_quat']).tolist()
                self.oMf_Lf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose() ), np.array(dict[index]['Landing_P']))
                self.ssp = 'Rf'
            else:
                self.ssp = []
        elif self.scenario == 'jump':
            quat = np.array(dict[index]['Landing_quat']).tolist()
            self.oMf_Rf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose() ), np.array(dict[index]['Landing_PR']))
            self.oMf_Lf = pin.SE3( pin.Quaternion(np.matrix(quat).transpose() ), np.array(dict[index]['Landing_PL']))

class Phases:
    def __init__(self, dict, scenario='walk'): # scenario = walk and jump
        self.dict = dict
        self.p = []
        
        self.size = len (self.dict)
        self.scenario = scenario

        for i in range(self.size):
            self.p.append(Phase(dict, i, 0, scenario))
            self.p.append(Phase(dict, i, 1, scenario))
            self.p.append(Phase(dict, i, 2, scenario))

    def print_phase(self, index, verbose = False):
        print ("Phase Index: ", index)
        if  self.p[index].type == 0:
            print ("Phase State: Initial DSP")
        elif self.p[index].type == 1:
            if self.scenario == 'walk':
                print ("Phase State: SSP with ", self.p[index].ssp, "supporting")
            elif self.scenario == 'jump':
                print ("Phase State: Jump")
        else:
            print ("Phase State: Final DSP")

        
        print ("Time sereis: ", self.p[index].time)

        if verbose:
            print ("COM sereis: ", self.setCOM(self.p[index].com_x, self.p[index].com_y, self.p[index].com_z))
            print ("Mom sereris: ", self.setMOM(self.p[index].Lx, self.p[index].Ly, self.p[index].Lz))

        if self.scenario == 'walk':
            if self.p[index].type == 1:
                if self.p[index].ssp == 'Lf':
                    print ("Final Landing Pos is ", self.p[index].oMf_Rf.translation)
                else:
                    print ("Final Landing Pos is ", self.p[index].oMf_Lf.translation)
        elif self.scenario == 'jump':
            print ("Final Landing rfoot Pos is ", self.p[index].oMf_Rf.translation)
            print ("Final Landing lfoot Pos is ", self.p[index].oMf_Lf.translation)
        
        print ("")
    def getContactType(self, index):
        if self.scenario == 'walk':
            if self.p[index].type is not 1:
                return self.p[index].type
            else:
                return self.p[index].ssp
        elif self.scenario == 'jump':
            return self.p[index].type

    def getFinalTime(self, index):
        if index < len(self.p)-1:
            return self.p[index+1].time[0]
        else:
            return self.p[index].time[-1]

    def getTimeSeries(self):
        ts = []
        for i in range(len(self.p)):
            ts = np.concatenate((ts, self.p[i].time))
        return ts
    def getPhase(self, index):
        return self.p[index]

    def getCOMSeries(self):
        comx = []
        comy = []
        comz = []
        for i in range(len(self.p)):
            comx = np.concatenate((comx, self.p[i].com_x))
            comy = np.concatenate((comy, self.p[i].com_y))
            comz = np.concatenate((comz, self.p[i].com_z))
        return self.setCOM(comx, comy, comz)

    def getCOMdotSeries(self):
        comx = []
        comy = []
        comz = []
        for i in range(len(self.p)):
            comx = np.concatenate((comx, self.p[i].com_x_dot))
            comy = np.concatenate((comy, self.p[i].com_y_dot))
            comz = np.concatenate((comz, self.p[i].com_z_dot))
        return self.setCOM(comx, comy, comz)


    def getMOMSeries(self):
        Lx = []
        Ly = []
        Lz = []
        for i in range(len(self.p)):
            Lx = np.concatenate((Lx, self.p[i].Lx))
            Ly = np.concatenate((Ly, self.p[i].Ly))
            Lz = np.concatenate((Lz, self.p[i].Lz))
        return self.setCOM(Lx, Ly, Lz)

    def getMOMdotSeries(self):
        Lx = []
        Ly = []
        Lz = []
        for i in range(len(self.p)):
            Lx = np.concatenate((Lx, self.p[i].Lx_dot))
            Ly = np.concatenate((Ly, self.p[i].Ly_dot))
            Lz = np.concatenate((Lz, self.p[i].Lz_dot))
        return self.setCOM(Lx, Ly, Lz)

    def setCOM(self, x, y, z):
        com = np.array(np.zeros((3, len(x))))
        com[0, :] = x
        com[1, :] = y
        com[2, :] = z
        return com

    def setMOM(self, x, y, z):
        mom = np.array(np.zeros((3, len(x))))
        mom[0, :] = x
        mom[1, :] = y
        mom[2, :] = z
        return mom    