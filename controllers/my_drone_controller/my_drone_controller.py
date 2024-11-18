#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""

import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, DRONE_CHANNEL, CPU_CHANNEL, PEDESTRIAN_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph
from classes_and_constans import get_graph
from functions import get_positions_graph_from_cpu

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, Camera, DistanceSensor, Display
import numpy as np
from math import cos, sin
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

import ast

FLYING_ATTITUDE = 1
MAX_FORWARD_SPEED = 0.5
MAX_SIDEWAY_SPEED = 0.5
MAX_YAW_RATE = 1
MAX_ALTITUDE = 3
SPEEDING_UNIT = 0.01

TIME_MARGIN = 500

graph = get_graph()
got_positions_from_cpu = False
on_node = True
on_track = False

class HandleCommands:
    def __init__(self, robot, rec_channel=DRONE_CHANNEL):
        self.timestep = int(robot.getBasicTimeStep())
        self.robot = robot
        self.receiver = robot.getDevice("receiver")
        self.receiver.setChannel(rec_channel)
        self.receiver.enable(int(robot.getBasicTimeStep()))
        self.emitter = robot.getDevice("emitter")
        self.emitter.setChannel(CPU_CHANNEL)
        self.display = robot.getDevice("display")  # Get the display device
        self.group_picked = None

        ## Initialize motors
        self.m1_motor = robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        ## Initialize Sensors
        self.imu = robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.gps = robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.camera = robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.range_front = robot.getDevice("range_front")
        self.range_front.enable(self.timestep)
        self.range_left = robot.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = robot.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = robot.getDevice("range_right")
        self.range_right.enable(self.timestep)

        ## Get keyboard
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        ## Initialize variables
        self.past_x_global = 0
        self.past_y_global = 0

        self.x_global = self.gps.getValues()[0]
        self.y_global = self.gps.getValues()[1]
        self.past_time = self.robot.getTime()

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = robot.getTime()
        self.sensor_read_last_time = robot.getTime()

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

        self.emitter = self.robot.getDevice("emitter")
        self.emitter.setChannel(DRONE_CHANNEL)
     
    def go_to_goal(self, x, y, z):
        # print(f'going to {x}, {y}, {z}')
        self.x_goal = x
        self.y_goal = y
        self.altitude_goal = z
        self.execute_configuration(self.x_goal, self.y_goal, self.altitude_goal)
        # print(f'goal reached {x}, {y}, {z}')
    
    def stay_in_position(self, seconds=-1):
        # print(f'staying in position {x_goal}, {y_goal}, {altitude_goal}')
        self.x_goal = self.gps.getValues()[0]
        self.y_goal = self.gps.getValues()[1]
        self.altitude_goal = MAX_ALTITUDE
        starting_time = robot.getTime()
        if seconds == -1:
             while robot.getTime() - starting_time < 0.1:
                self.execute_configuration(self.x_goal, self.y_goal, self.altitude_goal)
        else:
            while robot.getTime() - starting_time < 5:
                self.execute_configuration(self.x_goal, self.y_goal, self.altitude_goal)

        # print('finnished staying in position')

    def execute_configuration(self, x_goal, y_goal, altitude_goal):
        # Main loop executing the commands:
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0
        height_desired = altitude_goal

        pules_margin_time = 0
        reached_goal = False
        while robot.step(self.timestep) != -1:
            dt = robot.getTime() - self.past_time
            actual_state = {}

            ## Get sensor data
            roll = self.imu.getRollPitchYaw()[0]
            pitch = self.imu.getRollPitchYaw()[1]
            yaw = self.imu.getRollPitchYaw()[2]
            yaw_rate = self.gyro.getValues()[2]
            self.altitude = self.gps.getValues()[2]
            self.x_global = self.gps.getValues()[0]
            self.y_global = self.gps.getValues()[1]

            # Calculate global velocities
            v_x_global = (self.x_global - self.past_x_global) / dt
            v_y_global = (self.y_global - self.past_y_global) / dt

            ## Get body fixed velocities
            cosyaw = cos(yaw)
            sinyaw = sin(yaw)
            v_x = v_x_global * cosyaw + v_y_global * sinyaw
            v_y = -v_x_global * sinyaw + v_y_global * cosyaw

            ## Initialize values
            initial_state = {"pos": np.array([self.x_global, self.y_global, self.altitude]), "moment": np.array([v_x, v_y, yaw])}
            desired_state = {"pos": np.array([x_goal, y_goal, altitude_goal]), "moment": np.array([0, 0, 0])}

            desired_direction = desired_state["pos"] - initial_state["pos"]
            if np.linalg.norm(desired_direction) != 0:
                desired_direction = desired_direction / np.linalg.norm(desired_direction)

            distance = np.linalg.norm(initial_state["pos"] - desired_state["pos"])

            
            forward_distance = desired_state["pos"][0] - initial_state["pos"][0]
            sideways_distance = desired_state["pos"][1] - initial_state["pos"][1]

            # print("forward_distance: ", forward_distance , "sideways_distance: ", sideways_distance)
            

            # print(f"Current position: {initial_state['pos']}")
            # print(f"Desired position: {desired_state['pos']}")
            # # print(f"Current velocity: {initial_state['moment']}")
            # # print(f"Desired velocity: {desired_state['moment']}")
            # # # print("\n")
            # print(f"deseired_direction: {desired_direction}")
            # print(f"distance: {distance}")

            slowing_forward = False
            slowing_sideways = False

            if np.linalg.norm(initial_state["pos"] - desired_state["pos"]) > 0.1:
                # print("moving")
                if not slowing_forward:
                    if forward_distance > SPEEDING_UNIT and forward_desired < MAX_FORWARD_SPEED:
                        forward_desired += SPEEDING_UNIT
                    elif forward_distance < -SPEEDING_UNIT and forward_desired > -MAX_FORWARD_SPEED:
                        forward_desired -= SPEEDING_UNIT

                if not slowing_sideways:
                    if sideways_distance > SPEEDING_UNIT and sideways_desired < MAX_SIDEWAY_SPEED:
                        sideways_desired += SPEEDING_UNIT
                    elif sideways_distance < -SPEEDING_UNIT and sideways_desired > -MAX_SIDEWAY_SPEED:
                        sideways_desired -= SPEEDING_UNIT

                if yaw_desired - yaw > 0.1:
                    yaw_desired += 1
                elif yaw_desired - yaw < -0.1:
                    yaw_desired -= 1

                if altitude_goal - self.altitude > 0.1:
                    height_diff_desired = 0.1
                elif altitude_goal - self.altitude < -0.1 :
                    height_diff_desired = -0.1

                forward_desired = np.sign(forward_desired)*min(MAX_FORWARD_SPEED, np.abs(forward_desired))
                sideways_desired = np.sign(sideways_desired)*min(MAX_SIDEWAY_SPEED, np.abs(sideways_desired))

                if np.linalg.norm(initial_state["pos"][0] - desired_state["pos"][0]) < 0.3 and np.abs(forward_desired) > 10*SPEEDING_UNIT:
                    forward_desired -= np.sign(forward_desired)*SPEEDING_UNIT
                    slowing_forward = True
                    # print("slowing forward")
                
                if np.linalg.norm(initial_state["pos"][1] - desired_state["pos"][1]) < 0.3 and np.abs(sideways_desired) > 10*SPEEDING_UNIT:
                    slowing_sideways = True
                    sideways_desired -= np.sign(sideways_desired)*SPEEDING_UNIT
        
            else:
                # print("slowing down")
                if np.abs(forward_desired) > SPEEDING_UNIT:
                    forward_desired -= np.sign(forward_desired)*SPEEDING_UNIT*0.5
                else:
                    forward_desired = 0

                if np.abs(sideways_desired) > SPEEDING_UNIT:
                    sideways_desired -= np.sign(sideways_desired)*SPEEDING_UNIT*0.5
                else:
                    sideways_desired = 0

                # if yaw_desired - yaw > 0.1:
                #     yaw_desired += 0.1
                # elif yaw_desired - yaw < -0.1:
                #     yaw_desired -= 0.1
                # else:
                #     yaw_desired = 0
                
                # if altitude_goal - altitude > 0.1:
                #     height_diff_desired = 0.1
                # elif altitude_goal - altitude < -0.1:
                #     height_diff_desired = -0.1
                # else:
                #     height_diff_desired = 0

                if forward_desired == sideways_desired == 0:
                    reached_goal = True

            height_desired += height_diff_desired * dt

            # print(f"forward_desired: {forward_desired}, sideways_desired: {sideways_desired}, yaw_desired: {yaw_desired}, height_desired: {height_desired}")

            ## Example how to get sensor data
            ## range_front_value = range_front.getValue()
            ## cameraData = camera.getImage()

            ## PID velocity controller with fixed height
            
            motor_power = self.PID_CF.pid(dt, forward_desired, sideways_desired,
                                    yaw_desired, height_desired,
                                    roll, pitch, yaw_rate,
                                    self.altitude, v_x, v_y)

            self.m1_motor.setVelocity(-motor_power[0])
            self.m2_motor.setVelocity(motor_power[1])
            self.m3_motor.setVelocity(-motor_power[2])
            self.m4_motor.setVelocity(motor_power[3])

            self.past_time = robot.getTime()
            self.past_x_global = self.x_global
            self.past_y_global = self.y_global

            if self.group_picked is not None and pules_margin_time > TIME_MARGIN:
                self.send_follow_me_puls(self.group_picked)
                pules_margin_time = 0
            else:
                pules_margin_time += self.timestep

            if reached_goal:
                return

    def go_to(self, node_name):
        print(f'Drone: going to {node_name}')
        node = graph.get_node(node_name)
        cur_node = None
        cur_node_name = None
        cur_fisical_position = self.gps.getValues()
        smallest_distance = 1000000
        for n_name,n_node in graph.get_nodes().items():
            if n_node.fisical_position is not None:
                distance = np.linalg.norm(np.array(n_node.fisical_position) - np.array(cur_fisical_position))
                if distance < smallest_distance:
                    smallest_distance = distance
                    cur_node = n_node
                    cur_node_name = n_name
        
        # handle roundabouts
        # print(f"cur_node_name: {cur_node_name}, node_name: {node_name}")
        if set([cur_node_name, node_name]) == {"tl_1", "tl_2"}:
            # print("cur coord: ", cur_node.fisical_position)
            # print("node coord: ", node.fisical_position)
            # print("mid coord: ", (node.fisical_position[0] + cur_node.fisical_position[0])/2, (node.fisical_position[1]+3))
            self.go_to_goal((node.fisical_position[0] - 2), (node.fisical_position[1] + cur_node.fisical_position[1])/2, MAX_ALTITUDE)
        elif set([cur_node_name, node_name]) == {"bl_1", "bl_2"}:
            self.go_to_goal((node.fisical_position[0] + 2), (node.fisical_position[1] + cur_node.fisical_position[1])/2, MAX_ALTITUDE)
        elif set([cur_node_name, node_name]) == {"br_1", "br_2"}:
            self.go_to_goal((node.fisical_position[0] + 2), (node.fisical_position[1] + cur_node.fisical_position[1])/2, MAX_ALTITUDE)

        self.go_to_goal(node.fisical_position[0], node.fisical_position[1], MAX_ALTITUDE)
        self.emitter.setChannel(CPU_CHANNEL)
        message = (DRONE_CHANNEL, "reached_node", node_name)
        self.emitter.send(str(message).encode('utf-8'))
    
    def take_group_i(self, group_num):
        """Set text on the display."""
        self.display.setColor(0xFFFFFF)  # Set the text color to blue
        height = self.display.getHeight()
        width = self.display.getWidth()
        self.display.fillRectangle(0, 0, width, height)  # Fill the background
        self.display.setColor(0x0000FF)  # Set the text color to blue
        self.display.drawText(f" Group {group_num}", 5, 5)  # Display the text
        self.display.drawText(f"follow me!", 4, 25)  # Display the text

        self.group_picked = group_num
        messege = (DRONE_CHANNEL, group_num, "follow_me", self.gps.getValues()[0], self.gps.getValues()[1])
        # print(PEDESTRIAN_CHANNEL)
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        self.emitter.send(str(messege).encode('utf-8'))
        self.emitter.setChannel(CPU_CHANNEL)

        messege = (CPU_CHANNEL, "group_picked", f'g{group_num}')
        self.emitter.setChannel(CPU_CHANNEL)

    def send_follow_me_puls(self, group_num):
        messege = (DRONE_CHANNEL, group_num, "follow_me", self.gps.getValues()[0], self.gps.getValues()[1])
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        self.emitter.send(str(messege).encode('utf-8'))
        self.emitter.setChannel(CPU_CHANNEL)
    
    def drop_group_i(self, group_num, table_num):
        self.display.setColor(0xFFFFFF)  # Set the text color to blue
        height = self.display.getHeight()
        width = self.display.getWidth()
        self.display.fillRectangle(0, 0, width, height)  # Fill the background
        self.display.setColor(0x0000FF)  # Set the text color to blue
        self.display.drawText(f" Group {group_num}", 5, 5)  # Display the text
        self.display.drawText(f"this is", 10, 25)  # Display the text
        self.display.drawText(f"your table!", 0, 45)  # Display the text
        self.group_picked = None
        
        message = (DRONE_CHANNEL, group_num, "drop_group", table_num)
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        self.emitter.send(str(message).encode('utf-8'))
        self.emitter.setChannel(CPU_CHANNEL)

        message = (DRONE_CHANNEL, "group_dropped", f'g{group_num}', table_num)
        self.emitter.setChannel(CPU_CHANNEL)
        self.emitter.send(str(message).encode('utf-8'))

    def lift_off(self):
        print('Drone: lifiting off')
        while robot.step(self.timestep) != -1 and self.gps.getValues()[2] is None:
            pass

        height_desired = self.gps.getValues()[2]
        x_goal = self.gps.getValues()[0]
        y_goal = self.gps.getValues()[1]
        altitude_goal = MAX_ALTITUDE # Set your target altitude
        self.go_to_goal(x_goal, y_goal, altitude_goal)

    def listen_to_cpu(self):
        if self.receiver.getQueueLength() > 0:
            message = ast.literal_eval(self.receiver.getString())
            if message[0] == CPU_CHANNEL:
                if message[1] == "go_to":
                    self.go_to(message[2])
                elif message[1] == "take_group":
                    self.take_group_i(message[2])
                    print("Drone: take group")
                elif message[1] == "drop_group":
                    self.drop_group_i(message[2], message[3])
                else:
                    print("Unknown command")
            
            self.receiver.nextPacket()

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # Create HandleCommands instance
    command_handler = HandleCommands(robot)

    global got_positions_from_cpu

    #  get positions from cpu
    while robot.step(timestep) != -1 and not got_positions_from_cpu:
        # Check for incoming packets
        print(f"Queue length: {command_handler.receiver.getQueueLength()}")
        got_positions_from_cpu = get_positions_graph_from_cpu(command_handler.receiver, command_handler.emitter, graph, got_positions_from_cpu)

        if got_positions_from_cpu:
            print("Got positions from CPU")
            got_positions_from_cpu = True

    # lifiting off
    command_handler.lift_off()
    print("Drone Ready")

    # Main loop:
    while robot.step(timestep) != -1:
        command_handler.stay_in_position()
        command_handler.listen_to_cpu()
        

if __name__ == '__main__':
    robot = Robot()
    run_robot(robot)

    