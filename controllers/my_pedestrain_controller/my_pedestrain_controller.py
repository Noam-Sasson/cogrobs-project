# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container."""
from controller import Supervisor

import optparse
import math
import ast
import numpy as np

import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constans import DRONE_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, FOOD_ITEMS, CPU_CHANNEL, EXPECTED_PONDER_TIME, WAITER_CHANNEL, RATE_OF_ARRIVAL, EXPECTED_EATING_TIME


offsets = [np.array([0, 0]), np.array([0, -0.5]), np.array([-0.5, 0]), np.array([-0.5, -0.5])]
sitting_z_position = 0.86
standing_z_position = 1.26
vannishing_point = np.array([0, -12, 1.26])
vainnishing_rotaion = np.array([0, 0, 1, 0])

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        # print("Initializing Pedestrian")
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 1.15
        self.current_height_offset = 0
        self.time_step = 32
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        Supervisor.__init__(self)
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        self.reached_goal = False
        self.p_name = self.root_node_ref.getField("name").getSFString()
        self.reciever = self.getDevice("receiver")
        self.reciever.enable(self.time_step)
        self.emitter = self.getDevice("emitter")
        self.in_group = False
        self.is_sitting = False
        self.group_size = 0
        self.table_name = None
        self.is_order_manager = False
        # print("Pedestrian initialized")

    def run(self):
        """Set the Pedestrian pose and position."""
        # print("Running Pedestrian")
        if not hasattr(self, 'waypoints') or len(self.waypoints) < 2:
            opt_parser = optparse.OptionParser()
            opt_parser.add_option("--trajectory", default="", help="Specify the trajectory in the format [x1 y1, x2 y2, ...]")
            opt_parser.add_option("--speed", type=float, default=0.5, help="Specify walking speed in [m/s]")
            opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
            options, args = opt_parser.parse_args()
            if not options.trajectory or len(options.trajectory.split(',')) < 2:
                print("You should specify the trajectory using the '--trajectory' option.")
                print("The trajectory should have at least 2 points.")
                return
            if options.speed and options.speed > 0:
                self.speed = options.speed
            if options.step and options.step > 0:
                self.time_step = options.step
            else:
                self.time_step = int(self.getBasicTimeStep())
            point_list = options.trajectory.split(',')
            self.number_of_waypoints = len(point_list)
            self.waypoints = []
            for i in range(0, self.number_of_waypoints):
                self.waypoints.append([])
                self.waypoints[i].append(float(point_list[i].split()[0]))
                self.waypoints[i].append(float(point_list[i].split()[1]))
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # compute waypoints distance
        self.waypoints_distance = []
        for i in range(0, self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            y = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x * x + y * y))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x * x + y * y))

        starting_time = self.getTime()
        while not self.step(self.time_step) == -1 and not self.reached_goal:
            time = self.getTime() - starting_time

            current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
            # / 'total distance between way-point(X) and way-point(X+1)'
            ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))

            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                    self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)

            # adjust height
            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

            # move everything
            distance = time * self.speed
            relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                self.waypoints_distance[self.number_of_waypoints - 1]

            for i in range(0, self.number_of_waypoints):
                if self.waypoints_distance[i] > relative_distance:
                    break

            distance_ratio = 0
            if i == 0:
                distance_ratio = relative_distance / self.waypoints_distance[0]
            else:
                distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                    (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
            x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                (1 - distance_ratio) * self.waypoints[i][0]
            y = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                (1 - distance_ratio) * self.waypoints[i][1]
            root_translation = [x, y, self.ROOT_HEIGHT + self.current_height_offset]
            angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1],
                               self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0])
            rotation = [0, 0, 1, angle]

            # print(f"Current Position: {self.root_translation_field.getSFVec3f()}")
            # print(f"Target Position: {root_translation}")
            # print(f"Waypoints: {self.waypoints}")

            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)

            # Check if the pedestrian has reached the target position
            if math.isclose(root_translation[0], self.waypoints[-1][0], abs_tol=0.1) and \
               math.isclose(root_translation[1], self.waypoints[-1][1], abs_tol=0.1):
                # print("Reached the target position.")
                self.reached_goal = True

        self.reset_limb_configuration()

    def reset_limb_configuration(self):
        """Reset the limb configuration to a default standing position."""
        # print("Resetting limb configuration to default standing position.")
        standing_angles = [0.0] * self.BODY_PARTS_NUMBER  # Default standing angles
        for i in range(self.BODY_PARTS_NUMBER):
            self.joints_position_field[i].setSFFloat(standing_angles[i])

    def set_trajectory(self, trajectory):
        """Set the Pedestrian trajectory."""
        self.trajectory = trajectory

    def go_to_coordinate(self, x, y):
        """Set the target coordinates and update the trajectory."""
        # print(f"Setting target coordinates to ({x}, {y})")

        # Reset necessary parameters
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        self.reached_goal = False
        self.current_height_offset = 0
        self.waypoints = []
        self.waypoints_distance = []

        # Set the waypoints
        self.waypoints = [[self.root_translation_field.getSFVec3f()[0], self.root_translation_field.getSFVec3f()[1]], [x, y]]
        self.number_of_waypoints = len(self.waypoints)
        # print(f"len(self.waypoints): {len(self.waypoints)}")

        # Compute waypoints distance
        for i in range(0, self.number_of_waypoints):
            x_diff = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            y_diff = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x_diff * x_diff + y_diff * y_diff))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x_diff * x_diff + y_diff * y_diff))

        # print("Starting run")
        self.run()
        # print("Finished run")

    def listen_to_world(self):
        """Listen to the world for new coordinates."""
        max_itters = 10
        while self.reciever.getQueueLength() > 0 and max_itters > 0:
            max_itters -= 1
            message = ast.literal_eval(self.reciever.getString())
            if message not in [None, ""]:
                if message[0] == WORLD_GENERATOR_CHANNEL and message[1] == self.p_name:
                    x, y = message[-2], message[-1]
                    self.go_to_coordinate(x, y)
                    self.group_size = message[-3]
                    self.in_group = True
                    print(f"Pedestrian {self.p_name} is in group {message[-3]}")
                
                if message[0] == DRONE_CHANNEL and self.in_group:
                    my_group_num = int(self.p_name[1])
                    group_num = int(message[1])
                    if my_group_num == group_num:
                        if message[2] == "follow_me":
                            curent_position = self.root_translation_field.getSFVec3f()
                            if self.is_sitting:
                                curent_position[2] = standing_z_position
                                self.root_translation_field.setSFVec3f(curent_position)
                                self.is_sitting = False

                            p_index = int(self.p_name[-1]) - 1
                            x, y = message[-2] + offsets[p_index][0], message[-1] + offsets[p_index][1]
                            if (x - curent_position[0])**2 + (y - curent_position[1])**2 > 0.1:
                                self.go_to_coordinate(x, y)
                        if message[2] == "drop_group":
                            table_name = message[-1]
                            my_chair = table_name + f"_{int(self.p_name[-1])}"
                            my_chair_node = self.getFromDef(my_chair)
                            my_new_position = my_chair_node.getField("translation").getSFVec3f()
                            my_new_position[2] = sitting_z_position
                            my_new_rotation = my_chair_node.getField("rotation").getSFRotation()
                            self.root_translation_field.setSFVec3f(my_new_position)
                            self.root_rotation_field.setSFRotation(my_new_rotation)
                            self.is_sitting = True
                            self.table_name = table_name

                            if int(self.p_name[-1]) == self.group_size:
                                self.is_order_manager = True
                                self.ponder_and_call_waiter()
                                
                if message[0] == WAITER_CHANNEL and self.in_group and self.is_order_manager:
                    my_group_num = int(self.p_name[1])
                    group_num = int(message[1])
                    if my_group_num == group_num:
                        if message[2] == "make_order" and self.group_size == int(self.p_name[-1]):
                            self.order_food()
                        if message[2] == "order_delivered" and self.group_size  == int(self.p_name[-1]):
                            self.eat()

                if message[0] == PEDESTRIAN_CHANNEL and self.in_group:
                    if message[1] == "group_vanish" and int(message[-1]) == int(self.p_name[-1]):
                        self.in_group = False
                        self.reset_limb_configuration()
                        curent_position = self.root_translation_field.getSFVec3f()
                        curent_position[2] = standing_z_position
                        self.root_translation_field.setSFVec3f(vannishing_point)
                        self.root_rotation_field.setSFRotation(vainnishing_rotaion)
                        self.is_sitting = False

                        
                            
            self.reciever.nextPacket()

    def ponder_and_call_waiter(self):
        """Ponder and order for all pedestrians."""

        print(f"Pedestrian {self.p_name} is pondering")
        # Ponder
        start_time = self.getTime()
        end_time = self.sample_exponential(EXPECTED_PONDER_TIME)
        print(f"Pedestrian {self.p_name} pondering for {end_time} ms")
        while pedestrian.step(pedestrian.time_step) != -1 and self.getTime() - start_time < end_time:
            print(self.getTime() - start_time)
            pass
        
        print("sending for waiter")
        self.emitter.setChannel(CPU_CHANNEL)
        send_message = (PEDESTRIAN_CHANNEL, "ready_to_order", int(self.p_name[1]), self.table_name)
        self.emitter.send(str(send_message).encode('utf-8'))

    def order_food(self):
        # generate order
        dishes = list(FOOD_ITEMS.keys())
        dishes_chosen = np.random.choice(dishes, self.group_size, replace=False)
        self.emitter.setChannel(WAITER_CHANNEL)
        message = [PEDESTRIAN_CHANNEL]
        for dish in dishes_chosen:
            message.append(dish)
        
        message = tuple(message)
        self.emitter.send(str(message).encode('utf-8'))
        self.emitter.setChannel(CPU_CHANNEL)
        print(f"Pedestrian {self.p_name} ordered {dishes_chosen}")

    def eat(self):
        start_time = self.getTime()
        end_time = self.sample_exponential(EXPECTED_EATING_TIME)
        print(f"Pedestrian {self.p_name} eating for {end_time} ms")
        while pedestrian.step(pedestrian.time_step) != -1 and self.getTime() - start_time < end_time:
            pass

        # send message to CPU
        self.emitter.setChannel(CPU_CHANNEL)
        message = (CPU_CHANNEL, "finish_eating", int(self.p_name[-1]))
        self.emitter.send(str(message).encode('utf-8'))

        # make group vanish
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        message = (PEDESTRIAN_CHANNEL, "group_vanish", int(self.p_name[-1]))
        self.emitter.send(str(message).encode('utf-8'))
        self.in_group = False
        self.reset_limb_configuration()
        self.root_translation_field.setSFVec3f(vannishing_point)
        self.root_rotation_field.setSFRotation(vainnishing_rotaion)
        self.is_sitting = False
        


    
    def sample_exponential(self, rate):
        """Sample from an exponential distribution with the given rate."""
        return np.random.exponential(1/rate)

if __name__ == '__main__':
    pedestrian = Pedestrian()

    # Main loop
    while pedestrian.step(pedestrian.time_step) != -1:
        pedestrian.listen_to_world()
        
    
