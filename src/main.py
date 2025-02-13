from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
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
    "kp": 0.4,
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


    nb_samples = 500

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
        tau_comp = get_gravity_compensation(pendule1, 0.25)
        pendule1.set_joint_torque(0)
        torques[i] = pendule1.get_joint_torque()
        angles[i] = pendule1.get_joint_position()
        pendule1.set_joint_torque(tau_comp)
        print(tau_comp)
        tau_simu = tau_comp + torques[i]
        print("got", angles[i], "sending", angles[i])

        sim.setJointPosition(q1_handle, np.deg2rad(angles[i]))

        i+=1
        sim.step()

    pendule1.set_joint_torque(0)
    time.sleep(2)
    pendule1.shutdown()
    sim.stopSimulation()


    #plt.plot(range(0, nb_samples), torques)
    print(angles[-1])
    plt.plot(range(0, nb_samples), angles)
    plt.show()

if __name__ == "__main__":
    main()
