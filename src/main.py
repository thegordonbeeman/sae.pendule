from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pid import PID, PIDMonitor

np.set_printoptions(suppress=True)

USE_TOOL = False
USER_INPUT = False
if USE_TOOL:
    from resources.Tool import Tool

param_p1 = {
    "ts": 0.005,
    "length": 0.25,
    "weight": 0.200,
    "max_torque" : 1.0,
    "max_vel" : 200,
    "mul_weight": 1.0,
    "kp": 2,
    "kd": 0.1,
    "ki": 0.005,
    "fv":5e-3
}  # use a dictionnary to store the simulation parameters

param_p2 = {
    "ts": 0.005,
    "length": 0.143,
    "weight": 0.0,
    "max_torque" : 1.0,
    "max_vel" : 200,
    "mul_weight": 1.0,
    "kp": 1.2,
    "kd": 0.2,
    "ki": 0.05,
    "fv":5e-3
}  # use a dictionnary to store the simulation parameters

def get_gravity_compensation(angle, length, weight=0):
    return np.sin(np.deg2rad(angle))*length*weight*-9.81

def get_robot_data(pendulum):
    pendulum.set_joint_torque(0)
    position = pendulum.get_joint_position()
    torque = pendulum.get_joint_torque()
    return position, torque

def main():
    nb_samples = 1500

    pid = PID(
        param_p1["kp"],
        param_p1["ki"],
        param_p1["kd"]
        )
    pid_mon = PIDMonitor(pid, nb_samples)

    pendule_slave = RobotController("PENDULUM", USE_TOOL)
    pendule_master = RobotController("PENDULUM_MASTER", USE_TOOL)

    torques_master = np.empty(shape=nb_samples)
    angles_master = np.empty(shape=nb_samples)

    torques_slave = np.empty(shape=nb_samples)
    angles_slave = np.empty(shape=nb_samples)

    while True:
        a = input("mise à 0 ? (y): ")
        if a.lower() == 'y':
            break

    pendule_slave.init_offset() # Needed to put the robot at zero in the desired initial position
    pendule_master.init_offset()

    pendule_slave.set_joint_position(0)
    #pendule_master.set_joint_position(0)

    i = 0
    while i < nb_samples:
        time_prev = time.time()

        position_slave, torque_slave = get_robot_data(pendule_slave)
        position_master, torque_master = get_robot_data(pendule_master)
        tau_comp = get_gravity_compensation(position_slave, 0.25, 0.3)

        print(tau_comp)

        #Acquisition donnée maître et esclave
        #torques_master[i] = pendule_master.get_joint_torque()
        #angles_master[i] = pendule_master.get_joint_position()
        torques_slave[i] = pendule_slave.get_joint_torque()
        angles_slave[i] = pendule_slave.get_joint_position()
        
        #pid.set_SP(np.deg2rad(angles[i]))
        pid.set_SP(np.pi/2)
        pid.set_PV(np.deg2rad(pendule_slave.get_joint_position()))
        tau_pid = pid.update(param_p1["ts"])

        tau_jumeau = tau_comp + tau_pid

        pendule_slave.set_joint_torque(tau_jumeau)

        i+=1
        pid_mon.update()
        elapsed_time = time.time()-time_prev
        if (elapsed_time < param_p1['ts']):
            time.sleep(param_p1['ts']-elapsed_time)
            time_prev = time.time()

    pendule_slave.set_joint_torque(0)
    pendule_master.set_joint_torque(0)
    time.sleep(2)
    pendule_master.shutdown()
    pendule_slave.shutdown()

    pid_mon.graph_data(ts=param_p1["ts"])
    
if __name__ == "__main__":
    main()
