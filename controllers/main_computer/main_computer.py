"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, Receiver
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, CPU_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph, FOOD_ITEMS
from classes_and_constans import get_graph
import math
import ast

from collections.abc import Sequence, Mapping

graph = get_graph()
entities_channels = [DRONE_CHANNEL, WAITER_CHANNEL]

got_nodes_position = False
sent_possitions_to_entities = False
verified_entities_got_positions = False


def is_subscriptable(obj):
    """Check if an object is subscriptable."""
    return isinstance(obj, (Sequence, Mapping))

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

def verify_entities_got_positions(receiver, timestep):
    all_channels = set(entities_channels)
    collected_channels = set()
    
    global verified_entities_got_positions
    while robot.step(timestep) != -1 and not verified_entities_got_positions:
        message = receiver.getString()
        if message not in [None, ""]:
            collected_channels.add(int(message))

        print(all_channels, collected_channels)

        if all_channels == collected_channels:
            print("CPU: entities received positions successfully")
            verified_entities_got_positions = True
            break

        receiver.nextPacket()


class droneCommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, drone channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.world_state = world_state

    def go_to(self, position):
        message = (CPU_CHANNEL, "go_to", position)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to drone")

    def pick_up_customer_group(self, group_num):
        message = (CPU_CHANNEL, "take_group", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent pick_up_customer_group command to drone")

    def drop_off_customer_group(self, group_num, table):
        message = (CPU_CHANNEL, "drop_group", group_num, table)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent drop_off_customer_group command to drone")

class waiterCommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, waiter channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.current_position = world_state.waiter["position"]
        self.world_state = world_state
    
    def take_order(self, group_num):
        message = (CPU_CHANNEL,"take_order", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent take_order command to waiter")

    def go_to(self, node_to):
        message = (CPU_CHANNEL,"go_to", self.current_position, node_to)
        self.current_position = node_to
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to waiter")

    def pick_up_food(self, group_num):
        message = (CPU_CHANNEL,"pick_up_order", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent pick_up_food command to waiter")

    def deliver_food(self, group_num):
        message = (CPU_CHANNEL,"deliver_order", group_num)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent deliver_food command to waiter")

class cleanerCommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, cleaner channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.world_state = world_state
    
    def clean_table(self, table):
        pass

    def go_to(self, position):
        pass

class worldGeneratorcommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, world generator channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.world_state = world_state

    def kitchen_make_food(self, food):
        pass

class worldState:
    # Class variables for global states
    CLEAR, WAITING_TO_ORDER, READY_TO_ORDER, WAITING_FOR_FOOD, EATING, WAITHING_FOR_BILL, NEED_CLEANING = range(7) # tables states
    GROUP_OUTSIDE, GROUP_INSIDE = range(2) # groups states
    EMPTY_PLATTER, PLATTER_TO_BE_PLATED, PLATTER_WITH_FOOD = range(3) # kitchen states
    FOOD_ITEMS = FOOD_ITEMS
    MAX_MASSAGES = 10

    # a class to stores the state of the world (maybe even updates it)
    def __init__(self, receiver, available_kitchen_space=3):
        self.receiver = receiver
        self.groups_states = dict()
        self.tables_states = {"tbl_tl": self.CLEAR, "tbl_bl": self.CLEAR, "tbl_tr": self.CLEAR, "tbl_br": self.CLEAR}
        self.groups_to_tables = dict()
        self.kitchen_state = {i: self.EMPTY_PLATTER for i in range(available_kitchen_space)}
        self.orders = dict()

        self.drone = {"position": None, "carries": None}
        self.waiter = {"position": None, "carries": None}
        self.cleaner = {"position": None}

    def listen_to_entities(self, receiver):
        # print("CPU: Listening to entities")
        i = 0
        while receiver.getQueueLength() > 0 and i < self.MAX_MASSAGES:
            print(f"CPU: Listening to entities {i}")
            i += 1
            message = ast.literal_eval(receiver.getString())
            if message not in [None, ""] and is_subscriptable(message):
                if message[0] == WORLD_GENERATOR_CHANNEL:
                    if message[1] == "group_arrived":
                        group, group_size = message[2:]
                        print(f"CPU: Group {group} arrived with {group_size} members")
                        self.groups_states[group] = self.GROUP_OUTSIDE

                if message[0] == PEDESTRIAN_CHANNEL:
                    if message[1] == "ready_to_order":
                        group = message[2]
                        table = message[3]
                        self.tables_states[table] = self.READY_TO_ORDER
                        print(f"CPU: Group {group} is ready to order at table {table}")
                    if message[1] == "finished_eating":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.NEED_CLEANING
                        print(f"CPU: Group {group} finished eating at table {table}")

                if message[0] == DRONE_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.drone["position"] = node
                        print(f"CPU: Drone reached node {node}")
                    if message[1] == "group_dropped":
                        group = message[2]
                        table = message[3]
                        self.tables_states[table] = self.WAITING_TO_ORDER
                        self.groups_to_tables[group] = table
                        print(f"CPU: Group {group} dropped at table {table}")
                    if message[1] == "group_picked":
                        group = message[2]
                        self.drone["carries"] = group
                        print(f"CPU: Group {group} picked by drone")

                if message[0] == WAITER_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.waiter["position"] = node
                    if message[1] == "order_taken":
                        group = message[2]
                        order = message[3:]
                        self.orders[group] = order
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.WAITING_FOR_FOOD
                        print(f"CPU: Group {group} ordered {order}")
                    if message[1] == "picked_up_order":
                        group = message[2]
                        self.waiter["carries"] = group
                        print(f"CPU: Group {group} order picked up by waiter")
                    if message[1] == "order_delivered":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.EATING
                                             
            receiver.nextPacket()

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)
    world_state = worldState(receiver)

    world_state.drone["position"] = "tl_1"
    world_state.waiter["position"] = "k"

    drone_command_handler = droneCommnads(robot, DRONE_CHANNEL, timestep, world_state)
    waiter_command_handler = waiterCommnads(robot, WAITER_CHANNEL, timestep, world_state)
    cleaner_command_handler = cleanerCommnads(robot, CLEANER_CHANNEL, timestep, world_state)
    # define channels for communication

    while robot.step(timestep) != -1:
        print(f"got_nodes_position: {got_nodes_position}")
        print(f"sent_positions_to_entities: {sent_possitions_to_entities}")
        print(f"verified_entities_got_positions: {verified_entities_got_positions}")

        if not got_nodes_position:
            get_nodes_position(graph, receiver)

        if got_nodes_position and not sent_possitions_to_entities:
            send_possitions_to_entities(graph, emitter)

        if sent_possitions_to_entities and not verified_entities_got_positions:
            verify_entities_got_positions(receiver, timestep)

        if verified_entities_got_positions:
            world_state.listen_to_entities(receiver)
            break
    
    print("CPU: Ready to start the simulation")

    while robot.step(timestep) != -1 and len(world_state.groups_states) == 0:
        world_state.listen_to_entities(receiver)

    waiter_command_handler.go_to("tr_1")
    waiter_command_handler.go_to("mr_1")
    drone_command_handler.pick_up_customer_group(1)
    drone_command_handler.go_to("ml_1")
    drone_command_handler.go_to("ml")
    drone_command_handler.go_to("tbl_bl")
    drone_command_handler.drop_off_customer_group(1, "tbl_bl")
    waiter_command_handler.go_to("ml_2")
    waiter_command_handler.go_to("ml")
    waiter_command_handler.go_to("tbl_bl")
    waiter_command_handler.take_order(1)



            

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
