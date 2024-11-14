"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, Receiver
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, CPU_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph, FOOD_ITEMS, RATE_OF_PREP_TIME
from classes_and_constans import get_graph
import math
import ast

from planner import solve_problem
import time

from collections.abc import Sequence, Mapping

graph = get_graph()
entities_channels = [DRONE_CHANNEL, WAITER_CHANNEL, CLEANER_CHANNEL]

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
        if message not in [None, ""] and len(message) == 1:
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
        if self.world_state.tables_states[table] != self.world_state.CLEAR:
            print(f"CPU: Table {table} is not clear")
            return
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
        if len(self.world_state.kitchen_state) < self.world_state.available_kitchen_space:
            message = (CPU_CHANNEL,"take_order", group_num)
            self.emitter.setChannel(self.channel)
            self.emitter.send(str(message).encode('utf-8'))
            print(f"CPU: Sent take_order command to waiter")
        else:
            print(f"CPU: No available kitchen space")

    def go_to(self, node_to):
        message = (CPU_CHANNEL,"go_to", self.current_position, node_to)
        self.current_position = node_to
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to waiter")

    def pick_up_food(self, group_num):
        if self.world_state.waiter["carries"] is not None:
            print(f"CPU: Waiter already carries food")
            return
        if self.world_state.waiter["position"] != "k_in":
            print(f"CPU: Waiter is not in the kitchen")
            return
        if self.world_state.kitchen_state[group_num] != self.world_state.PLATTER_WITH_FOOD:
            print(f"CPU: Food is not ready for group {group_num}")
            return
        
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
        self.current_position = world_state.cleaner["position"]
        self.world_state = world_state
    
    def clean_table(self, table):
        if self.world_state.cleaner["position"] != table:
            print(f"CPU: Cleaner is not at table {table}")
            return
        if self.world_state.tables_states[table] != self.world_state.NEED_CLEANING:
            print(f"CPU: Table {table} is not dirty")
            return
        message = (CPU_CHANNEL, "clean_table", table)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent clean_table command to cleaner")

    def go_to(self, node_to):
        message = (CPU_CHANNEL,"go_to", self.current_position , node_to)
        self.current_position = node_to
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to cleaner")

class worldGeneratorcommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, world generator channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.world_state = world_state

    def kitchen_make_food(self, group, food):
        self.world_state.kitchen_state[group] = self.world_state.PLATTER_TO_BE_PLATED
        message = [CPU_CHANNEL, "make_food", group]
        for f in food:
            message += [f]
        message = tuple(message)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent make_food command to world generator")

class worldState:
    # Class variables for global states
    CLEAR, WAITING_TO_ORDER, READY_TO_ORDER, WAITING_FOR_FOOD, EATING, WAITHING_FOR_BILL, NEED_CLEANING = range(7) # tables states
    GROUP_OUTSIDE, GROUP_INSIDE = range(2) # groups states
    PLATTER_TO_BE_PLATED, PLATTER_WITH_FOOD = range(2) # kitchen states
    FOOD_ITEMS = FOOD_ITEMS
    MAX_MASSAGES = 10
    ORDER0 = "order0"
    ORDER1 = "order1"

    # a class to stores the state of the world (maybe even updates it)
    def __init__(self, robot, receiver, timestep=64, available_kitchen_space=2):
        self.receiver = receiver
        self.groups_states = dict()
        self.tables_states = {"tbl_tl": self.CLEAR, "tbl_bl": self.CLEAR, "tbl_tr": self.CLEAR, "tbl_br": self.CLEAR}
        self.groups_to_tables = dict()
        self.kitchen_state = dict()
        self.orders = dict() # all orders that ever been made - {order_id: {customer_id, table_id, profit, prep_time}}
        self.current_orders = {self.ORDER0:{'used': False, 'owner': None, 'is_ready': False, 'food':0},
                               self.ORDER1:{'used': False, 'owner': None, 'is_ready': False, 'food':0}} # current orders - {order_id: {customer_id, table_id, profit, prep_time}}
        self.total_profit = 0
        self.available_kitchen_space = available_kitchen_space
        self.world_generator = worldGeneratorcommnads(robot, WORLD_GENERATOR_CHANNEL, timestep, self)

        self.owner_to_order = dict()
        self.drone = {"position": None, "carries": None}
        self.waiter = {"position": None, "carries": None}
        self.cleaner = {"position": None}

        self.planner_timer = 0
        self.current_plan_idx = 0
        self.current_actions_sequence = []

        self.waiter_command_handler = waiterCommnads(robot, WAITER_CHANNEL, timestep, self)
        self.drone_command_handler = droneCommnads(robot, DRONE_CHANNEL, timestep, self)
        self.cleaner_command_handler = cleanerCommnads(robot, CLEANER_CHANNEL, timestep, self)

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
                        # {customer_id: {table_id, seated, ready_to_order, order_taken, served, eaten, party_size}}
                        self.groups_states[group] = {'status': self.GROUP_OUTSIDE, 'table': None, 'seated': False, 'ready_to_order': False, 'order_taken': False, 'served': False, 'eaten': False, 'party_size': group_size}
                    
                    if message[1] == "food_ready":
                        group = message[2]
                        self.current_orders[self.owner_to_order[group]]['is_ready'] = True
                        print(f"CPU: Group {group} food is ready")
                        self.kitchen_state[group] = self.PLATTER_WITH_FOOD
                    
                if message[0] == PEDESTRIAN_CHANNEL:
                    if message[1] == "ready_to_order":
                        group = message[2]
                        table = message[3]
                        self.tables_states[table] = self.READY_TO_ORDER
                        self.groups_states[group]['ready_to_order'] = True
                        print(f"CPU: Group {group} is ready to order at table {table}")
                    if message[1] == "finish_eating":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.NEED_CLEANING
                        self.groups_states[group]['eaten'] = True
                        self.groups_states[group]['status'] = self.GROUP_OUTSIDE
                        self.total_profit += self.orders[group]["profit"]
                        print(f"CPU: Group {group} finished eating at table {table}")
                        print(f"CPU: Total profit: {self.total_profit}")

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
                        self.groups_states[group]['table'] = table
                        self.groups_states[group]['seated'] = True
                        print(f"CPU: Group {group} dropped at table {table}")
                    if message[1] == "group_picked":
                        group = message[2]
                        self.drone["carries"] = group
                        self.groups_states[group]['status'] = self.GROUP_INSIDE
                        print(f"CPU: Group {group} picked by drone")

                if message[0] == WAITER_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.waiter["position"] = node
                    if message[1] == "order_taken":
                        group = message[2]
                        order = message[3:]
                        self.orders[group] = {'customer':group, 'table':self.groups_to_tables[group], 'profit': sum([self.FOOD_ITEMS[f] for f in order]), 'prep_time': len(order)*RATE_OF_PREP_TIME}
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.WAITING_FOR_FOOD
                        self.groups_states[group]['order_taken'] = True

                        self.current_orders[self.owner_to_order[group]]['food'] = self.orders[group]['profit']
                        # print(f"CPU: Group {group} ordered {order}")
                        # self.world_generator.kitchen_make_food(group, order) # check if order reaches kitchen
                        # print(f"CPU: Group {group} order sent to kitchen")
                    if message[1] == "picked_up_order":
                        group = message[2]
                        self.waiter["carries"] = group
                        print(f"CPU: Group {group} order picked up by waiter")
                    if message[1] == "order_delivered":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.EATING
                        self.groups_states[group]['served'] = True

                if message[0] == CLEANER_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.cleaner["position"] = node
                        print(f"CPU: Cleaner reached node {node}")
                    if message[1] == "table_cleaned":
                        table = message[2]
                        self.tables_states[table] = self.CLEAR
                        print(f"CPU: Table {table} cleaned")                         
            
            receiver.nextPacket()

    def get_world_state_dict(self):
        return {
            "groups_states": self.groups_states,
            "tables_states": self.tables_states,
            "groups_to_tables": self.groups_to_tables,
            "kitchen_state": self.kitchen_state,
            "orders": self.orders,
            "total_profit": self.total_profit,
            "available_kitchen_space": self.available_kitchen_space,
            "drone": self.drone,
            "waiter": self.waiter,
            "cleaner": self.cleaner
        }

    def send_for_planning(self):
        '''
        initial_state_dict should countain the following:
        - server_1_cur_loc: server_1 current location
        - cleaer_cur_loc: cleaner current location
        - host_cur_loc: host current location
        - g_following: group following the host
        - orders: orders status of the form {order_id: {customer_id, table_id, profit, prep_time}}
        - cust_out: list of customers that are outsid of the restaurant
        - cust_in: customers iniside status of the form {customer_id: {table_id, seated, ready_to_order, order_taken, served, eaten, party_size}}
        - tables: tables status of the form {table_id: {occupied, clean}}
        - revenue: current revenue
        '''

        if len(self.groups_states) == 0:
            print("CPU: No planning needed")
            return
        
        initial_state_dict = dict()
        initial_state_dict["server_1_cur_loc"] = self.waiter["position"]
        initial_state_dict["cleaer_cur_loc"] = self.cleaner["position"]
        initial_state_dict["host_cur_loc"] = self.drone["position"]
        initial_state_dict["g_following"] = self.drone["carries"]
        initial_state_dict["orders"] = self.orders
        initial_state_dict["cust_out"] = [k for k, v in self.groups_states.items() if v['status'] == self.GROUP_OUTSIDE and not v['eaten']] # groups that are outside and not eaten
        initial_state_dict["cust_in"] = {k: v for k, v in self.groups_states.items() if v['status'] != self.GROUP_OUTSIDE} # groups that are inside
        initial_state_dict["tables"] = self.tables_states # modify self.tables_states
        initial_state_dict["revenue"] = self.total_profit

        self.current_actions_sequence = solve_problem(initial_state_dict)

    def follow_planning(self):
        '''
        actions_sequence: [{action: a, start: s, duration:d}]
        '''

        if self.current_actions_sequence[self.current_plan_idx] >= self.planner_timer:
            pass
    
    def execut_plan(self, action):
        action_tup = str(action)[:-1].split("(")
        action_name = action_tup[0]
        action_args = action_tup[1].split(", ")
        if action_name == "move":
            entity = action_args[0]
            location_from = action_args[1]
            location_to = action_args[2]
            match entity:
                case "server_1":
                    self.waiter_command_handler.go_to(location_to)
                case "cleaner":
                    self.cleaner_command_handler.go_to(location_to)
                case "host":
                    self.drone_command_handler.go_to(location_to)
                case default:
                    print("CPU: Invalid entity")
        if action_name == "take_order":
            entity = action_args[0]
            group = action_args[1]
            table = action_args[2]
            order_slot = action_args[3]
            self.current_orders[order_slot]['used'] = True
            self.current_orders[order_slot]['owner'] = group
            self.current_orders[order_slot]['is_ready'] = False
            self.owner_to_order[group] = order_slot
            self.waiter_command_handler.take_order(group)
        if action_name == "take_food":
            entity = action_args[0]
            group = action_args[1]
            self.waiter_command_handler.pick_up_food(group)
        if action_name == "serve_food":
            entity = action_args[0]
            group = action_args[1]
            self.waiter_command_handler.deliver_food(group)
             
            
def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)
    world_state = worldState(robot, receiver, timestep)

    world_state.drone["position"] = "tl_1"
    world_state.waiter["position"] = "k_in"
    world_state.cleaner["position"] = "mr"

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

        # here update the world state.planner_timer
    
    print("CPU: Ready to start the simulation")

    # while robot.step(timestep) != -1 and len(world_state.groups_states) == 0:
    #     world_state.listen_to_entities(receiver)

    # waiter_command_handler.go_to("tr_1")
    # waiter_command_handler.go_to("mr_1")
    # drone_command_handler.pick_up_customer_group(1)
    # drone_command_handler.go_to("ml_1")
    # drone_command_handler.go_to("ml")
    # drone_command_handler.go_to("tbl_bl")
    # drone_command_handler.drop_off_customer_group(1, "tbl_bl")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("tbl_bl")
    # waiter_command_handler.take_order(1)
    
    # while robot.step(timestep) != -1 and world_state.tables_states["tbl_bl"] != world_state.WAITING_FOR_FOOD:
    #     world_state.listen_to_entities(receiver)

    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("mr_1")
    # waiter_command_handler.go_to("tr_1")
    # waiter_command_handler.go_to("k")
    # waiter_command_handler.go_to("k_in")

    # # do a check for waiter in the kitchen
    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "k_in": # make check for food ready
    #     world_state.listen_to_entities(receiver)
    
    # waiter_command_handler.pick_up_food(1)

    # waiter_command_handler.go_to("k")
    # waiter_command_handler.go_to("tr_1")
    # waiter_command_handler.go_to("mr_1")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("tbl_bl")

    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "tbl_bl":
    #     world_state.listen_to_entities(receiver)

    # waiter_command_handler.deliver_food(1)
    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("ml_1")
    
    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "ml_1":
    #     world_state.listen_to_entities(receiver)

    # cleaner_command_handler.go_to("mr_1")
    # cleaner_command_handler.go_to("ml_2")
    # cleaner_command_handler.go_to("ml")
    # cleaner_command_handler.go_to("tbl_bl")
    # cleaner_command_handler.clean_table("tbl_bl")

    # while robot.step(timestep) != -1 and world_state.tables_states["tbl_bl"] != world_state.CLEAR:
    #     world_state.listen_to_entities(receiver)



    # make full circle with waiter
    # waiter_command_handler.go_to("tr_1")
    # waiter_command_handler.go_to("mr_1")
    # # waiter_command_handler.go_to("br_1")
    # # waiter_command_handler.go_to("bl_2")
    # # waiter_command_handler.go_to("bl_1")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("tl_2")
    # waiter_command_handler.go_to("tl_1")
    # waiter_command_handler.go_to("ml_1")
    # waiter_command_handler.go_to("bl_1")
    # waiter_command_handler.go_to("bl_2")
    # waiter_command_handler.go_to("br_1")
    # waiter_command_handler.go_to("br_2")
    # waiter_command_handler.go_to("mr_2")
    # waiter_command_handler.go_to("tr_2")
    # waiter_command_handler.go_to("k")
    # waiter_command_handler.go_to("k_in")
    # waiter_command_handler.go_to("k")
    # waiter_command_handler.go_to("tr_1")
    # waiter_command_handler.go_to("mr_1")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("tbl_tl")
    # waiter_command_handler.go_to("ml")
    # waiter_command_handler.go_to("ml_2")
    # waiter_command_handler.go_to("mr_1")
    # waiter_command_handler.go_to("mr")
    # waiter_command_handler.go_to("tbl_tr")
    # waiter_command_handler.go_to("mr")
    # waiter_command_handler.go_to("tbl_br")
    explore_nodes_weights_with_dfs(graph, "tl_1", world_state.drone_command_handler.go_to, robot, timestep, world_state, receiver, world_state.drone)

    while robot.step(timestep) != -1:
        world_state.listen_to_entities(receiver)

def explore_nodes_weights_with_dfs(graph, node_name, go_to_func, robot, timestep, world_state, receiver, entity):
    """
    Explores all edges of a graph starting from the given node.

    :param graph: A dictionary representing the graph where keys are node names and values are lists of adjacent nodes.
    :param start_node: The starting node for the exploration.
    :return: A list of edges explored.
    """
    visited = set()
    edges = []
    node = graph.get_node(node_name)
    edges_weights = []
    def dfs(node):
        visited.add(node)
        for neighbor in graph.get_node(node.name).neighbours:
            edge = {node, neighbor}
            if edge not in edges:
                edges.append(edge)
                # calculate weight
                start_time = robot.getTime()
                go_to_func(neighbor.name)
                while robot.step(timestep) != -1 and entity["position"] != neighbor.name:
                    world_state.listen_to_entities(receiver)
                go_to_func(node.name)
                while robot.step(timestep) != -1 and entity["position"] != node.name:
                    world_state.listen_to_entities(receiver)
                end_time = robot.getTime()
                weight = (end_time - start_time)/2
                print(f"CPU: Weight between {node.name} and {neighbor.name} is {weight}")
                edges_weights.append((edge, weight))
            if neighbor not in visited:
                go_to_func(neighbor.name)
                while robot.step(timestep) != -1 and entity["position"] != neighbor.name:
                    world_state.listen_to_entities(receiver)
                dfs(neighbor)
                go_to_func(node.name)
                while robot.step(timestep) != -1 and entity["position"] != node.name:
                    world_state.listen_to_entities(receiver)

    dfs(node)

    for tup in edges_weights:
        e = list(tup[0])
        print(f'{(e[0].name, e[1].name)}: {tup[1]}')

    return edges

    


    

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
