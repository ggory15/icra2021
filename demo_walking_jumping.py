import pickle
from trajectory import interpolation as traj
from trajectory import phase as Phs
from curves import piecewise, polynomial, SE3Curve

''' For reading the trajectory from Jiyai '''
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
CurveSet = traj.Interpolation(Walk_phases, 0.0005)


''' For Talos Controller '''
import talos_conf_jump as conf
from tsid_biped_jump import TsidBiped
import vizutils
import numpy as np 
import pinocchio as pin
import copy
import tsid as TSID

np.set_printoptions(precision=4)

def get_COM_initial_traj(com, com_d=np.array([0.6, 0, 0.75])):
    c0 = com.copy()
    dc0 = np.zeros(3)
    ddc0 = np.zeros(3)
    t = 0
    phase = polynomial(c0, dc0, ddc0, com_d, dc0, ddc0, t, t+2)
    return phase

def get_foot_traj(oMi, oMf, stime, endtime, z_height =0.1):
    phase = []

    oMi_half= oMf.copy()
    oMi_half.translation = (oMf.translation + oMi.translation)/2.0
    oMi_half.translation[2] = z_height

    phase1 = SE3Curve(oMi, oMi_half, stime, (endtime+stime)/2.0)
    phase2 = SE3Curve(oMi_half, oMf, (endtime+stime)/2.0, endtime)

    phase.append(phase1)
    phase.append(phase2)
    return phase

def curvesToTSID(curves,t):
    # adjust t to bounds, required due to precision issues:
    if curves.min() > t > curves.min() - 1e-3:
        t = curves.min()
    if curves.max() < t < curves.max() + 1e-3:
        t = curves.max()
    sample = TSID.TrajectorySample(curves.dim())
    sample.pos(curves(t))
    sample.vel(np.zeros(3))
    return sample

def curveSE3toTSID(curve,t, computeAcc = True):
    # adjust t to bounds, required due to precision issues:
    if curve.min() > t > curve.min() - 1e-3:
        t = curve.min()
    if curve.max() < t < curve.max() + 1e-3:
        t = curve.max()
    sample = TSID.TrajectorySample(12,6)
    placement = curve.evaluateAsSE3(t)
    
    vel = curve.derivateAsMotion(t,1)
    sample.pos(SE3toVec(placement))
    sample.vel(MotiontoVec(vel))
    if computeAcc:
        acc = curve.derivateAsMotion(t, 2)
        sample.acc(MotiontoVec(acc))
    return sample

def footTrajcurves(phase, t):
    i = 0
    if phase[0].min() > t > phase[0].min() - 1e-3:
        t = phase[0].min()
    if phase[1].max() > t > phase[1].max() - 1e-3:
        t = phase[1].max()
    if phase[1].min() <= t:
        i = 1
   
    return curveSE3toTSID(phase[i], t)

def SE3toVec(M):
    v = np.zeros(12)
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

def MotiontoVec(M):
    v = np.zeros(6)
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v

tsid = TsidBiped(conf, conf.viewer)
i, t = 0, 0.0
q, v = tsid.q, tsid.v
cs = 0
phase_0_c = []
time_offset = 8000
sequence_change = True
swing_traj_L = []
swing_traj_R = []

while True:
    if t < time_offset / 2.0 * conf.dt:
        phase_0_c = get_COM_initial_traj(tsid.robot.com(tsid.formulation.data()))
        sampleCom = curvesToTSID(phase_0_c,t)
        tsid.comTask.setReference(sampleCom)
        i = i + 1
    elif t >= time_offset / 2.0 * conf.dt and t < time_offset * conf.dt:
        sampleCom = TSID.TrajectorySample(3)
        sampleCom.pos(np.array([0.6, 0, 0.75]))
        tsid.comTask.setReference(sampleCom)
        i = i + 1
    elif t >= time_offset * conf.dt and t <= Walk_phases.getFinalTime(len(Walk_phases.p)-1) + time_offset * conf.dt:
        if Walk_phases.getContactType(cs) == 0 or Walk_phases.getContactType(cs) == 2: # DSP
            if sequence_change and Walk_phases.getContactType(cs) == 0:
                # tsid.remove_contact_LF(Walk_phases.getFinalTime(cs)- (t-time_offset * conf.dt))
                # tsid.remove_contact_RF(Walk_phases.getFinalTime(cs) -(t-time_offset * conf.dt))
                sequence_change = False
            elif sequence_change and Walk_phases.getContactType(cs) == 2:
                tsid.add_contact_LF2(Walk_phases.p[cs].oMf_Lf)
                tsid.add_contact_RF2(Walk_phases.p[cs].oMf_Rf)
                sequence_change = False

            sampleCom = TSID.TrajectorySample(3)
            sampleCom.pos(CurveSet.com_traj[i-time_offset])  
            sampleCom.vel(CurveSet.com_dot_traj[i-time_offset])  
            tsid.comTask.setReference(sampleCom)

            sampleAM = TSID.TrajectorySample(3)
            sampleAM.pos(CurveSet.L_traj[i-time_offset])
            sampleAM.vel(CurveSet.L_dot_traj[i-time_offset])
            tsid.amTask.setReference(sampleAM)

        elif Walk_phases.getContactType(cs) == 1:
            sampleCom = TSID.TrajectorySample(3)
            sampleCom.pos(CurveSet.com_traj[i-time_offset])  
            sampleCom.vel(CurveSet.com_dot_traj[i-time_offset])  
            tsid.comTask.setReference(sampleCom)

            sampleAM = TSID.TrajectorySample(3)
            sampleAM.pos(CurveSet.L_traj[i-time_offset])
            sampleAM.vel(CurveSet.L_dot_traj[i-time_offset])
            tsid.amTask.setReference(sampleAM)
            if sequence_change:
                tsid.remove_contact_LF()
                tsid.remove_contact_RF()
                swing_traj_L = get_foot_traj(tsid.robot.framePosition(tsid.formulation.data(), tsid.LF), Walk_phases.getPhase(cs).oMf_Lf, t - time_offset * conf.dt, Walk_phases.getFinalTime(cs))
                swing_traj_R = get_foot_traj(tsid.robot.framePosition(tsid.formulation.data(), tsid.RF), Walk_phases.getPhase(cs).oMf_Rf, t - time_offset * conf.dt, Walk_phases.getFinalTime(cs))
                sequence_change = False

            sampleFoot_L = footTrajcurves(swing_traj_L,t-time_offset * conf.dt)
            tsid.leftFootTask.setReference(sampleFoot_L)
            sampleFoot_R = footTrajcurves(swing_traj_R,t-time_offset * conf.dt)
            tsid.rightFootTask.setReference(sampleFoot_R)

        if Walk_phases.getFinalTime(cs) < t - time_offset * conf.dt:
            cs += 1
            print ("Phase :", cs)
            sequence_change = True
        
        i = i + 1
    
    else:
        print (aa)
        sampleCom = TSID.TrajectorySample(3)
        sampleCom.pos(CurveSet.com_traj[i-1-time_offset])  
        sampleCom.vel(CurveSet.com_dot_traj[i-1-time_offset])  
        tsid.comTask.setReference(sampleCom)

        sampleAM = TSID.TrajectorySample(3)
        sampleAM.pos(CurveSet.L_traj[i-1-time_offset])
        sampleAM.vel(CurveSet.L_dot_traj[i-1-time_offset])
        tsid.amTask.setReference(sampleAM)    

        tsid.add_contact_LF2(Walk_phases.p[cs-1].oMf_Lf)
        tsid.add_contact_RF2(Walk_phases.p[cs-1].oMf_Rf)
        

    tsid.postureTask.setReference(tsid.trajPosture.computeNext())
    HQPData = tsid.formulation.computeProblemData(t, q, v)

    sol = tsid.solver.solve(HQPData)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break

    # tau = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)
    q, v = tsid.integrate_dv(q, v, dv, conf.dt)
    t =  t + conf.dt
    
    data = tsid.formulation.data()
    if tsid.contact_LF_active:
        J_LF = tsid.contactLF.computeMotionTask(0.0, q, v, data).matrix
    else:
        J_LF = np.zeros((0, tsid.model.nv))
    if tsid.contact_RF_active:
        J_RF = tsid.contactRF.computeMotionTask(0.0, q, v, data).matrix
    else:
        J_RF = np.zeros((0, tsid.model.nv))
    J = np.vstack((J_LF, J_RF))
    J_com = tsid.comTask.compute(t, q, v, data).matrix

    if i % conf.DISPLAY_N == 0:
        tsid.display(q)
        x_com = tsid.robot.com(tsid.formulation.data())
        x_com_ref = tsid.trajCom.getSample(t).pos()
        H_lf = tsid.robot.framePosition(tsid.formulation.data(), tsid.LF)
        H_rf = tsid.robot.framePosition(tsid.formulation.data(), tsid.RF)
        x_lf_ref = tsid.trajLF.getSample(t).pos()[:3]
        x_rf_ref = tsid.trajRF.getSample(t).pos()[:3]
        vizutils.applyViewerConfiguration(tsid.viz, 'world/com', x_com.tolist() + [0, 0, 0, 1.])
        vizutils.applyViewerConfiguration(tsid.viz, 'world/com_ref', x_com_ref.tolist() + [0, 0, 0, 1.])
        vizutils.applyViewerConfiguration(tsid.viz, 'world/rf', pin.SE3ToXYZQUATtuple(H_rf))
        vizutils.applyViewerConfiguration(tsid.viz, 'world/lf', pin.SE3ToXYZQUATtuple(H_lf))
        vizutils.applyViewerConfiguration(tsid.viz, 'world/rf_ref', x_rf_ref.tolist() + [0, 0, 0, 1.])
        vizutils.applyViewerConfiguration(tsid.viz, 'world/lf_ref', x_lf_ref.tolist() + [0, 0, 0, 1.])

    if i % 100 == 0 and t>4 and t <= Walk_phases.getFinalTime(len(Walk_phases.p)-1) + time_offset * conf.dt:
        print ('current com', tsid.robot.com(tsid.formulation.data()))
        print ('desired com', CurveSet.com_traj[i-time_offset])
        if tsid.formulation.checkContact(tsid.contactRF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactRF.name, sol)
            print("\tnormal force %s: %.1f"%(tsid.contactRF.name.ljust(20,'.'), tsid.contactRF.getNormalForce(f)))
        if tsid.formulation.checkContact(tsid.contactLF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactLF.name, sol)
            print("\tnormal force %s: %.1f"%(tsid.contactLF.name.ljust(20,'.'), tsid.contactLF.getNormalForce(f)))
