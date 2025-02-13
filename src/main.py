from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pid import PID, PIDMonitor

np.set_printoptions(suppress=True)

ROBOT = "PENDULUM" #define type of orobot PENDULUM/3DDL/SCARA
USE_TOOL = False
USER_INPUT = False
if USE_TOOL:
    from resources.Tool import Tool



param_p1 = {
    "ts": 0.005,
    "length": 0.25,
    "weight": 0.180,
    "max_torque" : 1.0,
    "max_vel" : 200,
    "mul_weight": 1.0,
    "kp": 1.2,
    "kd": 0.2,
    "ki": 0.05,
    "fv":5e-3
}  # use a dictionnary to store the simulation parameters

def get_gravity_compensation(pendulum, length, weight=0):
    pendulum.set_joint_torque(0)
    angle = pendulum.get_joint_position()
    return np.sin(np.deg2rad(angle))*length*weight*-9.81

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)

    # get objet handles
    q1_handle = sim.getObject("/q1")

    nb_samples = 1000

    pid = PID(
        param_p1["kp"],
        param_p1["ki"],
        param_p1["kd"]
        )
    pid_mon = PIDMonitor(pid, nb_samples)

    pendule1 = RobotController(ROBOT, USE_TOOL)
    torques = np.empty(shape=nb_samples)
    angles = np.empty(shape=nb_samples)
    while True:
        a = input("mise à 0 ? (y): ")
        if a.lower() == 'y':
            break

    pendule1.init_offset() # Needed to put the robot at zero in the desired initial position

    while True:
        a = input("setup simu ok ? (y): ")
        if a.lower() == 'y':
            break

    sim.startSimulation()
    

    pendule1.set_joint_position(0)

    i = 0
    while i < nb_samples:
        
        tau_comp = get_gravity_compensation(pendule1, 0.25, 0.2)
        pendule1.set_joint_torque(0)
        torques[i] = pendule1.get_joint_torque()
        angles[i] = pendule1.get_joint_position()
        pendule1.set_joint_torque(tau_comp)
        
        pid.set_SP(np.deg2rad(angles[i]))
        pid.set_PV(sim.getJointPosition(q1_handle))
        tau_pid = pid.update(param_p1["ts"])

        vitesse = sim.getJointVelocity(q1_handle)

        tau_simu = tau_comp + tau_pid - param_p1["fv"]*vitesse

        sim.setJointTargetForce(q1_handle, tau_simu)
        i+=1
        sim.step()
        pid_mon.update()

    pendule1.set_joint_torque(0)
    time.sleep(2)
    pendule1.shutdown()
    sim.stopSimulation()


    pid_mon.graph_data(ts=param_p1["ts"])
    

if __name__ == "__main__":
    main()
