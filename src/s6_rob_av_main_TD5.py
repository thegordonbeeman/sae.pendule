########################################################################
#           IMPORTANT              #
#  configure can before start. In the terminal run: 
#  sudo ip link set can0 up type can bitrate 1000000  <- This command is needed at every connection of the motor
########################################################################

#import asyncio
#from async_motor_controller import MotorController
from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np

ROBOT = "PENDULUM" #define type of orobot PENDULUM/3DDL/SCARA
USE_TOOL = False
USER_INPUT = False
if USE_TOOL:
    from resources.Tool import Tool

param = {
    "ts": 0.005,
    "length": 0.25,
    "weight": 0.180,
    "max_torque" : 1.0,
    "max_vel" : 200,
    "mul_weight": 1.0,
    "kp": 0.4,
}  # use a dictionnary to store the simulation parameters

couples = []

def main():
    robot = RobotController(ROBOT, USE_TOOL)
    # Wait for users input to start
    while True:
        a = input("mise à 0 ? (y): ")
        if a.lower() == 'y':
            break

    robot.init_offset() # Needed to put the robot at zero in the desired initial position 

    nb_samples = 200
    
    robot.set_joint_position(0)
    q_start = robot.get_joint_position()
    ee = 60 - q_start
    for k in range(nb_samples):
        
        tau = 0

        # Calculate tau_c
        #tau_c = np.sin(np.deg2rad(q)) * param["length"] * param["weight"] * param["mul_weight"] * -9.81
        q = robot.get_joint_position()
        ee = 60 - q

        tau_p = param["kp"] * ee #+ param["kd"] * de #+ param["ki"] * sum(e_all)/(param["ts"]*k+1)
        tau = tau_p

        time_prev = time.time()
        elapsed_time = time.time()-time_prev
        if (elapsed_time < param['ts']):
            time.sleep(param['ts']-elapsed_time)
            time_prev = time.time()

        # security before sending tau_c
        if(tau>param["max_torque"]):
            robot.shutdown()
            break

        # send tau_c
        robot.set_joint_torque(tau)

        # Read sensors
        if k < nb_samples - 1:
            robot.set_joint_torque(tau)

            q = robot.get_joint_position()
            '''print(robot.get_joint_torque())
            couples.append(robot.get_joint_torque())'''

        # security after reading sensors
        if(robot.get_joint_velocity()>param["max_vel"] or robot.get_joint_torque()>param["max_torque"]):
            robot.shutdown()
            break
        
        # add code the check if ts is over

        
    robot.set_joint_torque(0)
    time.sleep(2)
    robot.shutdown()

    print(couples)
#

if __name__ == "__main__":
    main()
