import pinocchio as pin
import numpy as np
np.set_printoptions(precision=4)
from curves import piecewise, polynomial

class Interpolation:
    def __init__(self, phases, frequency = 0.001):
        self.phases = phases
        self.time_ori = self.phases.getTimeSeries()

        self.com_ori = self.phases.getCOMSeries()
        self.L_ori = self.phases.getMOMSeries()
        self.com_dot_ori = self.phases.getCOMdotSeries()
        self.L_dot_ori = self.phases.getMOMdotSeries()        

        self.L_traj, self.L_dot_traj = self.getMOMtrajectory(frequency)
        self.com_traj, self.com_dot_traj = self.getCOMtrajectory(frequency)

    def getCOMtrajectory(self, frequency=0.001):
        com = []
        com_dot = []
        time = 0.0
        self.time_traj = []
        for i in range(len(self.time_ori) - 1):
            com_interpolation = polynomial(self.com_ori[:, i], self.com_dot_ori[:, i], self.com_ori[:, i+1], self.com_dot_ori[:, i+1], self.time_ori[i], self.time_ori[i+1])
            while time <= self.time_ori[i+1]:
                com.append(com_interpolation(time))
                com_dot.append(com_interpolation.derivate(time, 1))
                self.time_traj.append(time)
                time += frequency

        return com, com_dot    
    def getMOMtrajectory(self, frequency=0.001):
        mom = []
        mom_dot = []
        time = 0.0
        for i in range(len(self.time_ori) - 1):
            mom_interpolation = polynomial(self.L_ori[:, i], self.L_dot_ori[:, i], self.L_ori[:, i+1], self.L_dot_ori[:, i+1], self.time_ori[i], self.time_ori[i+1])
            while time <= self.time_ori[i+1]:
                mom.append(mom_interpolation(time))
                mom_dot.append(mom_interpolation.derivate(time, 1))
                time += frequency

        return mom, mom_dot 
