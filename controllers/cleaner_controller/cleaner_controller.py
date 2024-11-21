"""odometer_calculation controller."""


# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import os
import sys

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Camera, Receiver
import math
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, CLEANER_CHANNEL, CPU_CHANNEL, PEDESTRIAN_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph, RATE_OF_CLEANING_TIME
from classes_and_constans import get_graph
from functions import get_positions_graph_from_cpu
import numpy as np

import math
import ast

graph = get_graph()
got_positions_from_cpu = False
on_node = True
on_track = False

FRONT = 0
BACK = 3.14
SIDE = 1.57

class HandeCommands:
    def __init__(self, robot):
        # get the time step of the current world.
        self.robot = robot
        self.timestep = 64
        self.max_speed = -6.28*0.2

        # Create an instances of motors
        self.left_motor = robot.getDevice('motor_1')
        self.right_motor = robot.getDevice('motor_2')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        # get position sensor instances
        self.left_ps = robot.getDevice('ps_1')
        self.left_ps.enable(self.timestep)
        self.right_ps = robot.getDevice('ps_2')
        self.right_ps.enable(self.timestep)

        # get camera instance
        self.camera = robot.getDevice('color_sensor')
        self.camera.enable(self.timestep)

        # get ir sensor instances
        self.left_ir = robot.getDevice('ir_w_1')
        self.left_ir.enable(self.timestep)

        self.right_ir = robot.getDevice('ir_w_2')
        self.right_ir.enable(self.timestep)

        # Get the receiver device
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(self.timestep)

        # Get the emitter device
        self.emitter = robot.getDevice('emitter')

        # get gps instance
        self.front_gps = robot.getDevice('front_gps')
        self.front_gps.enable(self.timestep)
        self.back_gps = robot.getDevice('back_gps')
        self.back_gps.enable(self.timestep)

        # compute encoder unit
        self.wheel_radius = 0.4
        self.distance_between_wheels = 0.6

        self.wheel_circumference = 2.0 * 3.14 * self.wheel_radius
        self.encoder_unit = self.wheel_circumference / 6.28

        # robot position
        self.robot_pos = [0, 0, 0] # x, y, theta
        self.last_ps_values = [0, 0]
        
        self.ps_values = [0, 0]
        self.dis_values = [0, 0]
   
    def get_color_from_camera(self, image, width, height):
        # Process the image to detect color
        red_count = 0
        green_count = 0
        blue_count = 0
        orange_count = 0
        for x in range(width):
            for y in range(height):
                r = Camera.imageGetRed(image, width, x, y)
                g = Camera.imageGetGreen(image, width, x, y)
                b = Camera.imageGetBlue(image, width, x, y)
                if r > 200 and g < 100 and b < 100:  # Detect red color
                    red_count += 1
                elif g > 200 and r < 100 and b < 100:  # Detect green color
                    green_count += 1
                elif b > 200 and r < 100 and g < 100:  # Detect blue color
                    blue_count += 1
                elif r > 200 and g > 100 and b < 100:  # Detect orange color
                    orange_count += 1

        if red_count > (width * height * 0.1):  # If more than 10% of the image is red
            # print("Red color detected")
            return RED
        elif green_count > (width * height * 0.1):  # If more than 10% of the image is green
            # print("Green color detected")
            return GREEN
        elif blue_count > (width * height * 0.1):  # If more than 10% of the image is blue
            # print("Blue color detected")
            return BLUE
        elif orange_count > (width * height * 0.1):  # If more than 10% of the image is orange
            # print("Orange color detected")
            return ORANGE
        else:
            # print("No color detected")
            return NOCOLOR

    def update_sensor_data(self, left_ps, right_ps, last_ps_values, ps_values, dis_values, encoder_unit, distance_between_wheels, robot_pos):

        # Read the sensors:
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()

        print("----------------------------")
        print("Left ps: ", ps_values[0], "Right ps: ", ps_values[1])

        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 0.001:
                diff = 0 # to avoid noise
                ps_values[ind] = last_ps_values[ind]

            dis_values[ind] = diff * encoder_unit

        # compute linear and angular velocities
        v = (dis_values[0] + dis_values[1]) / 2.0 # linear velocity
        w = (dis_values[1] - dis_values[0]) / distance_between_wheels  # angular velocity

        dt = 1
        robot_pos[2] += w * dt

        vx = v * math.cos(robot_pos[2])
        vy = v * math.sin(robot_pos[2])

        robot_pos[0] += vx * dt
        robot_pos[1] += vy * dt

        print("Robot position: ", robot_pos)


        print("Left distance: ", dis_values[0], "Right distance: ", dis_values[1])

        for ind in range(2):
            last_ps_values[ind] = ps_values[ind]

    def make_turn(self, robot, angle, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep):
        leaniar_velocity = wheel_radius * max_speed

        start_time = robot.getTime()

        
        angle_of_rotation = angle
        rate_of_rotation = (2*leaniar_velocity) / distance_between_wheels
        duration_turn = abs(angle_of_rotation / rate_of_rotation)

        rot_start_time = start_time
        rot_end_time = rot_start_time + duration_turn


        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        turn_completed = False

        while robot.step(timestep) != -1 and not turn_completed:
            # print("Turning")
            current_time = robot.getTime()

            if current_time < rot_end_time:
                left_speed = -max_speed*np.sign(angle)
                right_speed = max_speed*np.sign(angle)
            else:
                turn_completed = True
                left_speed = 0
                right_speed = 0
        
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

    def go_forward(self, robot, distance, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep):
        vanila_speed = max_speed*0.25
        leaniar_velocity = wheel_radius * vanila_speed

        duration_side = abs(distance / leaniar_velocity)

        start_time = robot.getTime()

        lin_end_time = start_time + duration_side

        completed = False
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        while robot.step(timestep) != -1 and not completed:
            current_time = robot.getTime()

            if current_time < lin_end_time:
                left_speed = vanila_speed
                right_speed = vanila_speed
            else:
                completed = True
                left_speed = 0
                right_speed = 0
            
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

    def go_from_node_to_node(self, start_node, end_node):
        if self.front_gps is None or self.back_gps is None:
            return

        currnet_color = self.get_color_from_camera(self.camera.getImage(), self.camera.getWidth(), self.camera.getHeight())
        # print("Current color: ", currnet_color)

        while currnet_color == NOCOLOR:
            self.go_forward(self.robot, 0.05, self.left_motor, self.right_motor, self.max_speed, self.wheel_radius, self.distance_between_wheels, self.timestep)
            currnet_color = self.get_color_from_camera(self.camera.getImage(), self.camera.getWidth(), self.camera.getHeight())
            # print("Current color: ", currnet_color)

        rotations = [FRONT, SIDE, BACK]

        self.go_forward(self.robot, 0.2, self.left_motor, self.right_motor, self.max_speed, self.wheel_radius, self.distance_between_wheels, self.timestep)

        start_node = graph.get_node(start_node)
        end_node = graph.get_node(end_node)

        # print("Start node position: ", start_node.fisical_position)
        # print("start node neighbors: ", [n.name for n in start_node.get_neighbours()])
        # print("End node position: ", end_node.fisical_position)
        # print("Robot position: ", np.array(self.back_gps.getValues()))

        robot_direction = np.array(self.front_gps.getValues()[:2]) - np.array(self.back_gps.getValues()[:2])
        robot_direction = robot_direction / np.linalg.norm(robot_direction)
        robot_pos = np.array(self.back_gps.getValues()[:2])

        goal_direction = np.array(end_node.fisical_position[:2]) - np.array(start_node.fisical_position[:2])
        goal_direction = goal_direction / np.linalg.norm(goal_direction)

        # print("Robot direction: ", robot_direction)
        # print("Goal direction: ", goal_direction)     

        angle = np.arccos(np.dot(robot_direction, goal_direction)/(np.linalg.norm(robot_direction)*np.linalg.norm(goal_direction)))
        # print("Angle: ", angle)
        # print("Angle: ", angle)
        closest_angle = min([rot for rot in rotations], key=lambda x: abs(x - angle))

        # print("Original closest angle: ", closest_angle)
        # find direction of left/right rotation
        angle_sign = 1
        if np.cross(robot_direction, goal_direction) >= 0:
            angle_sign = -1

        if len(start_node.get_neighbours()) == 2 and closest_angle == SIDE:
            if start_node.name == "tr_1" and end_node.name == "k" and closest_angle == SIDE:
                if angle_sign == 1:
                    closest_angle = FRONT
                else:
                    closest_angle = BACK
            elif start_node.name == "tr_2" and end_node.name == "k" and closest_angle == SIDE:
                if angle_sign == 1:
                    closest_angle = BACK
                else:
                    closest_angle = FRONT
            elif start_node.name == "br_2":
                if angle_sign == 1:
                    closest_angle = FRONT
                else:
                    closest_angle = BACK
            elif start_node.name == "bl_1":
                if angle_sign == 1:
                    closest_angle = BACK
                else:
                    closest_angle = FRONT
            elif start_node.name == "tl_1":
                if angle_sign == 1:
                    closest_angle = FRONT
                else:
                    closest_angle = BACK
            elif start_node.name == "tl_2":
                if angle_sign == 1:
                    closest_angle = BACK
                else:
                    closest_angle = FRONT
            else:
                closest_angle = FRONT
            

        if len(start_node.get_neighbours()) == 3 and len(start_node.name) >= 2 and len(end_node.name) >= 2 and start_node.name[:2] == end_node.name[:2] \
            and start_node.name[:2] not in ["mr", "ml"]:
            if closest_angle == SIDE:
                if start_node.name == "br_1":
                    if angle_sign == -1:
                        closest_angle = FRONT
                    else:
                        closest_angle = BACK
                if start_node.name == "bl_2":
                    if angle_sign == 1:
                        closest_angle = FRONT
                    else:
                        closest_angle = BACK
            elif closest_angle == FRONT:
                if start_node.name == "br_1":
                    angle_sign = 1
                    closest_angle = SIDE
                if start_node.name == "bl_2":
                    closest_angle = SIDE
                    angle_sign = -1
                            

        closest_angle = closest_angle * angle_sign

        # if len(start_node.get_neighbours()) == 3 and closest_angle == FRONT:
        #     if start_node.name == "bl_2":
        #         closest_angle = SIDE*angle_sign
        #     if start_node.name == "br_1":
        #         closest_angle = SIDE*angle_sign

        # print("Closest Angle: ", closest_angle)
        if np.abs(closest_angle) > 0.1:
            self.make_turn(self.robot, closest_angle, self.left_motor, self.right_motor, self.max_speed, self.wheel_radius, self.distance_between_wheels, self.timestep)
        self.go_forward(self.robot, 0.1, self.left_motor, self.right_motor, self.max_speed, self.wheel_radius, self.distance_between_wheels, self.timestep)
        reached_other_node = False
        while self.robot.step(self.timestep) != -1 and not reached_other_node:
            color = self.get_color_from_camera(self.camera.getImage(), self.camera.getWidth(), self.camera.getHeight())

            # print("Color: ", color)
            if color!= currnet_color and color != NOCOLOR:
                reached_other_node = True
                left_speed = 0
                right_speed = 0
            else:
                left_ir_value = self.left_ir.getValue()
                right_ir_value = self.right_ir.getValue()

                # print("Left IR: ", left_ir_value, "Right IR: ", right_ir_value)

                if (left_ir_value > right_ir_value) and (900 < left_ir_value <= 1000):
                    # print("turn right")
                    left_speed = -self.max_speed
                    right_speed = self.max_speed
                elif (right_ir_value > left_ir_value) and (900 < right_ir_value <= 1000):
                    # print("turn left")
                    left_speed = self.max_speed
                    right_speed = -self.max_speed
                else:
                    left_speed = self.max_speed
                    right_speed = self.max_speed

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

        # print("Reached other node")
        self.dont_move()
        self.emitter.setChannel(CPU_CHANNEL)
        message = (CLEANER_CHANNEL, "reached_node", end_node.name)
        self.emitter.send(str(message).encode('utf-8'))

    def dont_move(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def listen_to_cpu(self):
        if self.receiver.getQueueLength() > 0:
            message = ast.literal_eval(self.receiver.getString())
            if message[0] == CPU_CHANNEL:
                if message[1] == "clean_table":
                    self.clean_table(message[2])
                elif message[1] == "go_to":
                    print(message)
                    self.go_from_node_to_node(message[2], message[3])
                    print("Cleaner: arrived at destination", message[3])
                elif message[1] == "kill":
                    self.kill()
            
            self.receiver.nextPacket()

    def sample_exponential(self, rate):
        """Sample from an exponential distribution with the given rate."""
        return np.random.exponential(1/rate)
    
    def clean_table(self, table):
        start_time = self.robot.getTime()
        end_time = start_time + self.sample_exponential(RATE_OF_CLEANING_TIME)/1000

        while self.robot.step(self.timestep) != -1 and self.robot.getTime() < end_time:
            pass

        self.emitter.setChannel(CPU_CHANNEL)
        message = (CLEANER_CHANNEL, "table_cleaned", table)
        self.emitter.send(str(message).encode('utf-8'))
        
    def kill(self):
        while self.receiver.getQueueLength() > 0:
            self.receiver.nextPacket()

        self.emitter.setChannel(CPU_CHANNEL)
        message = (CLEANER_CHANNEL, "killed")
        self.emitter.send(str(message).encode('utf-8'))

def run_robot(robot):
    command_handler = HandeCommands(robot)
    command_handler.dont_move()
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(command_handler.timestep) != -1:
        # Check for incoming packets
        global got_positions_from_cpu

        if not got_positions_from_cpu:
            got_positions_from_cpu = get_positions_graph_from_cpu(command_handler.receiver, command_handler.emitter, graph, got_positions_from_cpu)
            print("Cleaner Ready")

        if got_positions_from_cpu:
            command_handler.listen_to_cpu()

        # command_handler.update_sensor_data(left_ps, right_ps, last_ps_values, ps_values, dis_values, encoder_unit, distance_between_wheels, robot_pos)

        
        
       


        

    # Enter here exit cleanup code.

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)

    
