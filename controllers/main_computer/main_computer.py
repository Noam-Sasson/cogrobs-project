"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, Receiver
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL, WORLD_GENERATOR_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph
from classes_and_constans import get_graph
import math
import ast

graph = get_graph()
entities_channels = [DRONE_CHANNEL]

got_nodes_position = False
sent_possitions_to_entities = False
verified_entities_got_positions = False

def get_nodes_position(graph ,receiver):
    nodes = graph.get_nodes()
    len_nodes = len(nodes)
    non_null_count = 0
    while receiver.getQueueLength() > 0:
            message = receiver.getString()
            if message not in [None, ""]:
                non_null_count += 1
                data = ast.literal_eval(message)  # Safely parse the string back into a tuple
                name, position = data
                graph.set_node_position(name, position)
                print(f"CPU Received position from {name}: {position}")
                receiver.nextPacket()
    
    print(non_null_count, len_nodes)
    if non_null_count == len_nodes:
        print("CPU: All nodes position received from sensors")
        global got_nodes_position
        got_nodes_position = True

def send_possitions_to_entities(graph, emitter):
    nodes = graph.get_nodes()

    for entity_channel in entities_channels:
        for node_name, node in nodes.items():
            message = (node_name, node.fisical_position)
            emitter.setChannel(entity_channel)
            emitter.send(str(message).encode('utf-8'))
        print(f"All nodes position sent to {entity_channel}")
    
    print("CPU: All nodes position sent to entities")
    global sent_possitions_to_entities
    sent_possitions_to_entities = True

def verify_entities_got_positions(receiver):
    all_channels = set(entities_channels)
    collected_channels = set()
    for channel in entities_channels:
        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            if message not in [None, ""]:
                collected_channels.add(int(channel))
            receiver.nextPacket()
    
    if all_channels == collected_channels:
        print("CPU: entities received positions successfully")
        global verified_entities_got_positions
        verified_entities_got_positions = True


class droneCommnads:
    def __init__(self, robot, channel, timestep):
        """gets the main computer, drone channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot

    def go_to(self, position):
        message = ("go_to", position)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to drone")

    def pick_up_customer_group(self, group_num):
        message = ("pick_up_customer_group", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent pick_up_customer_group command to drone")

    def drop_off_customer_group(self, group_num):
        message = ("drop_off_customer_group", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent drop_off_customer_group command to drone")

class waiterCommnads:
    def __init__(self, robot, channel, timestep):
        """gets the main computer, waiter channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
    
    

class cleanerCommnads:
    pass

class worldState:
    # Class variables for global states
    CLEAR, WAITING_TO_ORDER, WAITING_FOR_FOOD, EATING, WAITHING_FOR_BILL, NEED_CLEANING = range(6) # tables states
    GROUP_OUTSIDE, GROUP_INSIDE = range(2) # groups states
    EMPTY_PLATTER, PLATTER_TO_BE_PLATED, PLATTER_WITH_FOOD = range(3) # kitchen states
    FOOD_ITEMS = {"dish1": 35, "dish2": 60, "dish3": 45, "dish4": 50, "dish5": 15}
    MAX_MASSAGES = 10

    # a class to stores the state of the world (maybe even updates it)
    def __init__(self, receiver, available_kitchen_space=3):
        self.receiver = receiver
        self.groups_states = dict()
        self.tables_states = {"tbl_tl": self.CLEAR, "tbl_bl": self.CLEAR, "tbl_tr": self.CLEAR, "tbl_br": self.CLEAR}
        self.kitchen_state = {i: self.EMPTY_PLATTER for i in range(available_kitchen_space)}

    def listen_to_entities(self, receiver):
        # print("CPU: Listening to entities")
        i = 0
        while receiver.getQueueLength() > 0 and i < self.MAX_MASSAGES:
            i += 1
            message = ast.literal_eval(receiver.getString())
            if message not in [None, ""]:
                if message[0] == WORLD_GENERATOR_CHANNEL:
                    if message[1] == "group_arrived":
                        group, group_size = message[2:]
                        print(f"CPU: Group {group} arrived with {group_size} members")
                        self.groups_states[group] = self.GROUP_OUTSIDE
                else:
                    print(f"CPU: Received message from entity: {message}")
            
            receiver.nextPacket()

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)
    world_state = worldState(receiver)

    # define channels for communication
    
    while robot.step(timestep) != -1:
        # print(f"got_nodes_position: {got_nodes_position}")
        # print(f"sent_positions_to_entities: {sent_possitions_to_entities}")
        # print(f"verified_entities_got_positions: {verified_entities_got_positions}")

        if not got_nodes_position:
            get_nodes_position(graph, receiver)

        if got_nodes_position and not sent_possitions_to_entities:
            send_possitions_to_entities(graph, emitter)

        if sent_possitions_to_entities and not verified_entities_got_positions:
            verify_entities_got_positions(receiver)

        if verified_entities_got_positions:
            world_state.listen_to_entities(receiver)
            

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
