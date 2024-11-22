"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, Receiver
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, CPU_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph, FOOD_ITEMS, RATE_OF_PREP_TIME, DISH_OCUNT, RATE_OF_CLEANING_TIME, RATE_OF_EATING_TIME, RATE_OF_PONDER_TIME
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
        self.world_state.drone["on_edge"] = True
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to drone {position}")

    def pick_up_customer_group(self, group_name):
        message = (CPU_CHANNEL, "take_group", group_name)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent pick_up_customer_group command to drone")

    def drop_off_customer_group(self, group, table):
        if self.world_state.tables_states[table] != self.world_state.CLEAR:
            print(f"CPU: Table {table} is not clear")
            return
        message = (CPU_CHANNEL, "drop_group", group, table)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent drop_off_customer_group command to drone")

    def kill(self):
        message = (CPU_CHANNEL, "kill")
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent kill command to drone")

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
    
    def take_order(self, group_name, dish):
        if not self.world_state.dishes[dish]["used"]:
            message = (CPU_CHANNEL,"take_order", group_name, dish)
            self.emitter.setChannel(self.channel)
            self.emitter.send(str(message).encode('utf-8'))
            print(f"CPU: Sent take_order command to waiter")
        else:
            print(f"CPU: dish {dish} is already in use")

    def go_to(self, node_to):
        message = (CPU_CHANNEL,"go_to", self.current_position, node_to)
        self.world_state.waiter["on_edge"] = True
        self.current_position = node_to
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to waiter {node_to}")

    def pick_up_food(self, group_name):
        if self.world_state.waiter["carries"] is not None:
            print(f"CPU: Waiter already carries food")
            return
        if self.world_state.waiter["position"] != "k_in":
            print(f"CPU: Waiter is not in the kitchen")
            return
        if self.world_state.kitchen_state[group_name] != self.world_state.PLATTER_WITH_FOOD:
            print(f"CPU: Food is not ready for group {group_name}")
            return
        
        message = (CPU_CHANNEL,"pick_up_order", group_name)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent pick_up_food command to waiter")

    def deliver_food(self, group_name):

        message = (CPU_CHANNEL,"deliver_order", group_name)
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent deliver_food command to waiter")

    def kill(self):
        message = (CPU_CHANNEL, "kill")
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent kill command to waiter")
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
        if self.world_state.Tables[table] != self.world_state.cleaner["position"]:
            print(f"CPU: Cleaner is not at table {table}")
            return
        if self.world_state.tables_states[table] != self.world_state.NEED_CLEANING:
            print(f"CPU: Table {table} is not dirty")
            return
        message = (CPU_CHANNEL, "clean_table", table)
        self.world_state.current_orders[table]['cleaning_time'] = (1/RATE_OF_CLEANING_TIME)/1000 # convert to seconds
        self.world_state.current_orders[table]['time_bais'] = 0
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        self.world_state.cleaner["cleaning"] = table
        print(f"CPU: Sent clean_table command to cleaner")

    def go_to(self, node_to):
        message = (CPU_CHANNEL,"go_to", self.current_position , node_to)
        self.world_state.cleaner["on_edge"] = True
        self.current_position = node_to
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent go_to command to cleaner {node_to}")

    def kill(self):
        message = (CPU_CHANNEL, "kill")
        self.emitter.setChannel(self.channel)
        self.emitter.send(str(message).encode('utf-8'))
        print(f"CPU: Sent kill command to cleaner")

class worldGeneratorcommnads:
    def __init__(self, robot, channel, timestep, world_state):
        """gets the main computer, world generator channel and timestep"""
        self.channel = channel
        self.emitter = robot.getDevice('emitter')
        self.receiver = robot.getDevice('receiver')
        self.receiver.enable(timestep)
        self.main_computer = robot
        self.world_state = world_state

    def kitchen_make_food(self, group, food, dish):
        self.world_state.kitchen_state[group] = self.world_state.PLATTER_TO_BE_PLATED
        self.world_state.dishes[dish]["used"] = True
        self.world_state.dishes[dish]["used_by"] = self.world_state.groups_to_tables[group]
        self.world_state.orders[group]["dish"] = dish
        self.world_state.current_orders[self.world_state.groups_to_tables[group]]['in_making'] = True
        self.world_state.current_orders[self.world_state.groups_to_tables[group]]['time_bais'] = 0
        message = [CPU_CHANNEL, "make_food", group, dish]
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
    DISH_OCUNT = DISH_OCUNT

    Tables = {"table_tl": "tbl_tl", "table_bl": "tbl_bl", "table_tr": "tbl_tr", "table_br": "tbl_br"}

    # a class to stores the state of the world (maybe even updates it)
    def __init__(self, robot, receiver, timestep=64, available_kitchen_space=2):
        self.robot = robot
        self.receiver = receiver
        self.time_step = timestep
        self.groups_states = dict()
        self.current_groups_count = 0
        self.tables_states = {k : self.CLEAR for k in self.Tables.keys()}
        self.groups_to_tables = dict()
        self.tables_to_groups = dict()
        self.kitchen_state = dict()
        self.needs_killing = 0
        self.orders = dict() # all orders that ever been made - {order_id: {customer_id, table_id, profit, prep_time}}
        self.dishes = {f"dish_{i}": {"used": False, "used_by": None} for i in range(1, self.DISH_OCUNT+1)}
        self.current_orders = {table_name: {"food_order": 0, "food_prep_time": 0, "can_be_taken": False, "in_making": False
                                            , "eating_time": (1/RATE_OF_EATING_TIME)/1000, "cleaning_time": (1/RATE_OF_CLEANING_TIME)/1000, "pondering_time": (1/RATE_OF_PONDER_TIME)/1000,
                                            "ready_to_order": False, "order_taken": False, "served": False, "occupied": False, "clean": True, "time_bais": 0} for table_name in self.tables_states.keys()}
        self.total_profit = 0
        self.available_kitchen_space = available_kitchen_space
        self.world_generator = worldGeneratorcommnads(robot, WORLD_GENERATOR_CHANNEL, timestep, self)

        # self.customer_to_table = dict()
        self.drone = {"position": 'tl_1', "carries": None, 'on_edge': False, "on_seating": False, "on_taking_customers": False}
        self.waiter = {"position": 'k_in', "carries": None, 'on_edge': False, "on_taking_order": False, "on_serving": False}
        self.cleaner = {"position": 'mr', "cleaning": None, 'on_edge': False}

        self.planner_start_time = 0
        self.current_plan_idx = 0
        self.current_actions_sequence = []
        self.planning_is_needed = False

        self.group_in_planning_size = 0

        self.waiter_command_handler = waiterCommnads(robot, WAITER_CHANNEL, timestep, self)
        self.drone_command_handler = droneCommnads(robot, DRONE_CHANNEL, timestep, self)
        self.cleaner_command_handler = cleanerCommnads(robot, CLEANER_CHANNEL, timestep, self)
        self.world_generator_command_handler = worldGeneratorcommnads(robot, WORLD_GENERATOR_CHANNEL, timestep, self)

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
                        dish = message[3]
                        self.current_orders[self.groups_to_tables[group]]['is_ready'] = True
                        self.current_orders[self.groups_to_tables[group]]['in_making'] = False
                        self.current_orders[self.groups_to_tables[group]]['can_be_taken'] = True
                        print(f"CPU: Group {group} food is ready")
                        self.kitchen_state[group] = self.PLATTER_WITH_FOOD
                    
                if message[0] == PEDESTRIAN_CHANNEL:
                    if message[1] == "ready_to_order":
                        group = message[2]
                        table = message[3]
                        self.tables_states[table] = self.READY_TO_ORDER
                        self.groups_states[group]['ready_to_order'] = True
                        self.current_orders[table]['ready_to_order'] = True
                        print(f"CPU: Group {group} is ready to order at table {table}")
                    if message[1] == "finish_eating":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.NEED_CLEANING
                        self.groups_states[group]['eaten'] = True
                        self.groups_states[group]['status'] = self.GROUP_OUTSIDE
                        self.current_orders[table]['occupied'] = False
                        self.current_orders[table]['clean'] = False
                        self.current_orders[table]['cleaning_time'] = (1/RATE_OF_CLEANING_TIME)/1000 # convert to seconds
                        self.total_profit += self.orders[group]["profit"]
                        print(f"CPU: Group {group} finished eating at table {table}")
                        print(f"CPU: Total profit: {self.total_profit}")

                if message[0] == DRONE_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.drone["position"] = node
                        self.drone["on_edge"] = False
                        print(f"CPU: Drone reached node {node}")
                    if message[1] == "group_dropped":
                        group = message[2]
                        table = message[3]
                        self.tables_states[table] = self.WAITING_TO_ORDER
                        self.groups_to_tables[group] = table
                        self.tables_to_groups[table] = group
                        self.groups_states[group]['table'] = table
                        self.groups_states[group]['seated'] = True
                        self.current_orders[table]['occupied'] = True
                        self.current_orders[table]['pondering_time'] = (1/RATE_OF_PONDER_TIME)/1000 # convert to seconds
                        self.current_orders[table]['time_bais'] = 0
                        self.drone["carries"] = None
                        self.drone["on_seating"] = False
                        print(f"CPU: Group {group} dropped at table {table}")
                    if message[1] == "group_picked":
                        group = message[2]
                        self.drone["carries"] = group
                        self.groups_states[group]['status'] = self.GROUP_INSIDE
                        self.drone["on_taking_customers"] = False
                        print(f"CPU: Group {group} picked by drone")
                    if message[1] == "killed":
                        self.needs_killing -= 1
                        print(f"CPU: drone killed")

                if message[0] == WAITER_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.waiter["position"] = node
                        self.waiter["on_edge"] = False
                    if message[1] == "order_taken":
                        group = message[2]
                        dish = message[3]
                        order = message[4:]
                        self.orders[group] = {'customer':group, 'table':self.groups_to_tables[group], 'profit': sum([self.FOOD_ITEMS[f] for f in order]), 'prep_time': len(order)*(1/RATE_OF_PREP_TIME)/1000, 'food_items': order, 'dish': None}
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.WAITING_FOR_FOOD
                        self.groups_states[group]['order_taken'] = True
                        self.current_orders[table]['order_taken'] = True
                        self.dishes[dish]["used"] = True
                        self.dishes[dish]["used_by"] = self.groups_to_tables[group]
                        self.current_orders[self.groups_to_tables[group]]['food_order'] = self.orders[group]['profit']
                        self.current_orders[self.groups_to_tables[group]]['food_prep_time'] = self.orders[group]['prep_time']

                        self.waiter['on_taking_order'] = False
                        print(f"CPU: Group {group} ordered {order}")
                        # self.world_generator.kitchen_make_food(group, order) # check if order reaches kitchen
                        # print(f"CPU: Group {group} order sent to kitchen")
                    if message[1] == "picked_up_order":
                        group = message[2]
                        self.waiter["carries"] = self.groups_to_tables[group]
                        dish = self.orders[group]['dish']
                        self.dishes[dish]["used"] = False
                        self.dishes[dish]["used_by"] = None
                        print(f"CPU: Group {group} order picked up by waiter")
                    if message[1] == "order_delivered":
                        group = message[2]
                        table = self.groups_to_tables[group]
                        self.tables_states[table] = self.EATING
                        self.groups_states[group]['served'] = True
                        self.current_orders[table]['served'] = True
                        self.current_orders[table]['eating_time'] = (1/RATE_OF_EATING_TIME)/1000 # convert to seconds
                        self.waiter["carries"] = None
                        self.current_orders[table]['time_bais'] = 0
                        self.waiter['on_serving'] = False
                    if message[1] == "killed":
                        self.needs_killing -= 1
                        print(f"CPU: waiter killed")

                if message[0] == CLEANER_CHANNEL:
                    if message[1] == "reached_node":
                        node = message[2]
                        self.cleaner["position"] = node
                        self.cleaner["on_edge"] = False
                        print(f"CPU: Cleaner reached node {node}")
                    if message[1] == "table_cleaned":
                        table = message[2]
                        self.tables_states[table] = self.CLEAR

                        self.cleaner["cleaning"] = None

                        # reseting table state
                        self.current_orders[table]['clean'] = True
                        self.current_orders[table]['cleaning_time'] = (1/RATE_OF_CLEANING_TIME)/1000 # convert to seconds
                        self.current_orders[table]['time_bais'] = 0
                        self.current_orders[table]['in_making'] = False
                        self.current_orders[table]['can_be_taken'] = False
                        self.current_orders[table]['ready_to_order'] = False
                        self.current_orders[table]['order_taken'] = False
                        self.current_orders[table]['served'] = False
                        self.current_orders[table]['occupied'] = False
                        self.current_orders[table]['food_order'] = 0
                        self.current_orders[table]['food_prep_time'] = 0
                        self.current_orders[table]['eating_time'] = (1/RATE_OF_EATING_TIME)/1000
                        self.current_orders[table]['pondering_time'] = (1/RATE_OF_PONDER_TIME)/1000

                        print(f"CPU: Table {table} cleaned")
                    if message[1] == "killed":
                        self.needs_killing -= 1
                        print(f"CPU: cleaner killed")                      
            
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
        - server_1: {position, holds}
        - cleaer: {position, cleaning}
        - host: {position, seating_customers}
        - orders: {table_name: {food_order, food_prep_time, can_be_taken, in_making, eating_time, cleaning_time, pondering_time, ready_to_order, order_taken , served, occupied, clean, time_bais}}
        - customers: {customer_id: {following, table, seated, eaten, party_size, in_rest}}
        - dishes: {dish: {used, used_by}}
        - revenue: current revenue

        needs to be added to planner
        - cleaning_time: cleaning time per table
        - ordering_time: ordering time per table
        - eating_time: eating time per table
        - prep_time: prep time per table
        '''
        
        print("CPU: In send_for_planning")
        
        done_planning = False
        
        customers_needs_planning = {k: v for k, v in self.groups_states.items() if not (v['status'] == self.GROUP_OUTSIDE and v['eaten'] == True)} # all groups that need to be served
        
        customers = dict()
        
        keys = sorted(list(customers_needs_planning.keys()), key=lambda x: int(x[-1]))

        if len(keys) > 4:
            keys = keys[:4]

        for k in keys:
            customers[k] = customers_needs_planning[k]

        self.group_in_planning_size = len(customers)

        if len(customers) == 0:
            print("CPU: No customers to serve")
            self.planning_is_needed = False
            return
        else:
            self.planning_is_needed = True
        
        initial_state_dict = dict()
        initial_state_dict["server_1"] = dict()
        initial_state_dict["server_1"]["position"] = self.waiter["position"]
        initial_state_dict["server_1"]["holds"] = self.waiter["carries"]
        initial_state_dict["cleaner"] = dict()
        initial_state_dict["cleaner"]["position"] = self.cleaner["position"]
        initial_state_dict["cleaner"]["cleaning"] = True if self.cleaner["cleaning"] is not None else False
        initial_state_dict["host"] = dict()
        initial_state_dict["host"]["position"] = self.drone["position"]
        initial_state_dict["host"]["seating_customers"] = True if self.drone["carries"] is not None else False

        initial_state_dict["customers"] = dict()
        for k, v in customers.items():
            initial_state_dict["customers"][k] = dict()
            initial_state_dict["customers"][k]['following'] = True if self.drone["carries"] == k else False
            initial_state_dict["customers"][k]['table'] = v['table']
            initial_state_dict["customers"][k]['seated'] = v['seated']
            initial_state_dict["customers"][k]['in_rest'] = True if self.groups_states[k]['status'] == self.GROUP_INSIDE else False
            initial_state_dict["customers"][k]['eaten'] = v['eaten']
            initial_state_dict["customers"][k]['party_size'] = v['party_size']

        initial_state_dict["dishes"] = dict()
        for dish in self.dishes:
            initial_state_dict["dishes"][dish] = dict()
            initial_state_dict["dishes"][dish] ['used'] = self.dishes[dish]['used']
            initial_state_dict["dishes"][dish] ['used_by'] = self.dishes[dish]['used_by']
        
        initial_state_dict["orders"] = dict()

        for k, v in self.current_orders.items():
            '''
            initial_state_dict["customers"][k]['ready_to_order'] = v['ready_to_order']
            initial_state_dict["customers"][k]['order_taken'] = v['order_taken']
            initial_state_dict["customers"][k]['served'] = v['served']
            '''
            initial_state_dict["orders"][k] = dict()
            initial_state_dict["orders"][k]['food_order'] = v['food_order']
            initial_state_dict["orders"][k]['food_prep_time'] = v['food_prep_time']
            initial_state_dict["orders"][k]['can_be_taken'] = v['can_be_taken']
            initial_state_dict["orders"][k]['in_making'] = v['in_making']
            initial_state_dict["orders"][k]['eating_time'] = v['eating_time']
            initial_state_dict["orders"][k]['cleaning_time'] = v['cleaning_time']
            initial_state_dict["orders"][k]['pondering_time'] = v['pondering_time']
            initial_state_dict["orders"][k]['ready_to_order'] = v['ready_to_order']
            initial_state_dict["orders"][k]['order_taken'] = v['order_taken']
            initial_state_dict["orders"][k]['served'] = v['served']
            initial_state_dict["orders"][k]['occupied'] = v['occupied']
            initial_state_dict["orders"][k]['clean'] = v['clean']
            initial_state_dict["orders"][k]['time_bais'] = v['time_bais']

            # time bais is emmited when replanning in the midle of one of following durative actions:
            # - pondering_time
            # - eating_time
            # - cleaning_time
            # - prep_time

            # additional tasks:
            # set bais to 0 if durative action is done - v
            # we replan only when "plan fails" - aka a stochastic mandatory condition is not met yet.
            
            # using bais when planning:

            # if (not v['ready_to_order']) and v['occupied']:
            #     initial_state_dict["orders"][k]['pondering_time'] = max(v['pondering_time'] - v['time_bais'], 1/2*(1/RATE_OF_PONDER_TIME)/1000)
            # elif v['in_making']:
            #     initial_state_dict["orders"][k]['prep_time'] = max(v['food_prep_time'] - v['time_bais'], 1/2*(1/RATE_OF_PREP_TIME)/1000)
            # elif v['served'] and v['clean']:
            #     initial_state_dict["orders"][k]['eating_time'] = max(v['eating_time'] - v['time_bais'], 1/2*(1/RATE_OF_EATING_TIME)/1000)
            # elif (not v['clean']) and self.cleaner["cleaning"] == k:
            #         initial_state_dict["orders"][k]['cleaning_time'] = max(v['cleaning_time'] - v['time_bais'], 1/2*(1/RATE_OF_CLEANING_TIME)/1000)
        

        initial_state_dict["tables"] = dict()
        for k, v in self.Tables.items():
            initial_state_dict["tables"][k] = dict()
            initial_state_dict["tables"][k]['position'] = v

        initial_state_dict["revenue"] = self.total_profit

        print("CPU: Sending for planning")
        current_best_solution = solve_problem(initial_state_dict.copy())

        for _ in range(4):
            new_solution = solve_problem(initial_state_dict.copy())
            if new_solution[-1]["start"] + new_solution[-1]["duration"] < current_best_solution[-1]["start"] + current_best_solution[-1]["duration"]:
                current_best_solution = new_solution
        
        for action in current_best_solution:
            print(action['start'], action['action'], action['duration'])

        self.current_actions_sequence = current_best_solution

        print(self.current_actions_sequence[0])
        self.planner_start_time = self.robot.getTime()
        print(f"CPU: Planning started at {self.planner_start_time}")
        self.current_plan_idx = 0

        print("CPU: Planning done")

    def follow_planning(self):
        '''
        actions_sequence: [{action: a, start: s, duration:d}]
        '''
        if self.planning_is_needed:
            current_planning_time = (self.robot.getTime() - self.planner_start_time)
            # print(f"CPU: Current planning time: {current_planning_time}")
            # print(current_planning_time, self.current_actions_sequence[self.current_plan_idx]['start'])
            if self.current_plan_idx < len(self.current_actions_sequence) and int(self.current_actions_sequence[self.current_plan_idx]['start']) <= current_planning_time:
                action = self.current_actions_sequence[self.current_plan_idx]['action']
                self.current_plan_idx += 1 # before replanning in execut_action !!!
                self.execut_action(action)
    
    # def update_planning_baises(self):
    #     if self.planning_is_needed:
    #         for table, order in self.current_orders.items():
    #             order['time_bais'] += self.timestep
                
 
    def execut_action(self, action):
        action_tup = str(action).split("(")
        action_name = action_tup[0]
        action_args = action_tup[1][:-1].split(", ")

        # TODO: add more replanning conditions
        # TODO: if move action repetets itself - replan
        # TODO: if action landmark was made (s.a food picked up) - replan
        # TODO: if going to a node that is already occupied - replan

        needs_replanning = False
        replanning_reasons = []

        print(f"CPU: Executing action {action_name} with args {action_args}")
        if action_name == 'take_customers':
            group = action_args[1]
            self.drone["on_taking_customers"] = True
            self.drone_command_handler.pick_up_customer_group(group)
        if action_name == 'seat_customers':
            table = action_args[1]
            group = action_args[2]
            if self.tables_states[table] != self.CLEAR:
                replanning_reasons.append(f"Table {table} is not clear")
            else:
                self.drone["on_seating"] = True
                self.drone_command_handler.drop_off_customer_group(group, table)
            
            replanning_reasons.append(f"Seated group {group} at table {table}")
            needs_replanning = True
        if action_name == 'clean_table':
            table = action_args[1]
            if self.tables_states[table] != self.NEED_CLEANING:
                replanning_reasons.append(f"Table {table} is not dirty yet, group still eating")
                needs_replanning = True
            else:
                self.cleaner_command_handler.clean_table(table)
        if action_name == 'take_order':
            table = action_args[1]
            group = self.tables_to_groups[table]
            dish = action_args[2]
            if self.tables_states[table] != self.READY_TO_ORDER:
                replanning_reasons.append(f"Table {table} is not ready to order")
            elif self.dishes[dish]["used"]:
                replanning_reasons.append(f"Dish {dish} is still in use")
                needs_replanning = True
            else:
                self.waiter['on_taking_order'] = True
                self.waiter_command_handler.take_order(group, dish)

            replanning_reasons.append(f"Waiter taking {group} order with {dish}")
            needs_replanning = True
        if action_name == 'make_order':
            table = action_args[0]
            dish = action_args[1]
            group = self.tables_to_groups[table]
            
            while self.robot.step(self.time_step) != -1 and group not in self.orders.keys():
                self.listen_to_entities(self.receiver)
            self.world_generator.kitchen_make_food(group, self.orders[group]['food_items'], dish)
            
        if action_name == 'take_food':
            table = action_args[1]
            group = self.tables_to_groups[table]
            if not self.current_orders[self.groups_to_tables[group]]['can_be_taken']:
                replanning_reasons.append(f"Food is not ready for group {group}")
                needs_replanning = True
            else:
                self.waiter_command_handler.pick_up_food(group)
               
        if action_name == 'serve_food':
            table = action_args[1]
            group = self.tables_to_groups[table]
            self.waiter_command_handler.deliver_food(group)
            self.waiter['on_serving'] = True
        if action_name == 'host_move':
            node = action_args[2]

            if node in self.Tables.values():
                if self.cleaner["position"] == node:
                    replanning_reasons.append(f"Cleaner is at table {node}")
                    needs_replanning = True
                else:
                    self.drone_command_handler.go_to(node)
            else:
                self.drone_command_handler.go_to(node)
        if action_name == 'server_move':
            node = action_args[2]
            from_node = action_args[1]

            if from_node != self.waiter["position"]:
                replanning_reasons.append(f"Waiter is not at node {from_node}")
                needs_replanning = True
            else:
                self.waiter_command_handler.go_to(node)

        if action_name == 'cleaner_move':
            if self.cleaner["cleaning"] is not None:
                replanning_reasons.append(f"Cleaner is cleaning table {self.cleaner['cleaning']}")
                needs_replanning = True
            elif self.cleaner["position"] != action_args[1]:
                replanning_reasons.append(f"Cleaner is not at node {action_args[1]}")
                needs_replanning = True
            else:
                node = action_args[2]
                self.cleaner_command_handler.go_to(node)
        if action_name == 'go_home':
            node = action_args[1]
            self.drone_command_handler.go_to('tl_1')

        customers_needs_planning = {k: v for k, v in self.groups_states.items() if not (v['status'] == self.GROUP_OUTSIDE and v['eaten'] == True)} # all groups that need to be served
        
        if len(customers_needs_planning)>=4 and self.group_in_planning_size < 4:
            print("CPU: Groups count changed")
            self.current_groups_count = len(self.groups_states)

            replanning_reasons.append("Groups count changed")
            needs_replanning = True
        
        if needs_replanning:
            print("CPU: ----------------- replanning reasons -----------------")
            for reason in replanning_reasons:
                print(reason)
                print(customers_needs_planning.keys())
            print("CPU: -----------------------------------------------------")
                
            while self.robot.step(self.time_step) != -1 and (self.waiter['on_edge'] or self.drone['on_edge'] or self.cleaner['on_edge'] or self.drone['on_seating'] or self.waiter['on_taking_order'] or self.waiter['on_serving']):
                self.listen_to_entities(self.receiver)

            self.drone_command_handler.kill()
            self.waiter_command_handler.kill()
            self.cleaner_command_handler.kill()

            self.needs_killing = 3
            while self.robot.step(self.time_step) != -1 and self.needs_killing > 0:
                if self.receiver.getQueueLength() > 0:
                    self.listen_to_entities(self.receiver)
                
                # TODO: check if works even after changes
                
            print(f"CPU: finnished killing")
                
            self.send_for_planning()

     
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

    # world_state.waiter_command_handler.go_to("tr_1")
    # world_state.waiter_command_handler.go_to("mr_1")
    # world_state.drone_command_handler.pick_up_customer_group(1)
    # world_state.drone_command_handler.go_to("ml_1")
    # world_state.drone_command_handler.go_to("ml")
    # world_state.drone_command_handler.go_to("tbl_bl")
    # world_state.drone_command_handler.drop_off_customer_group(1, "tbl_bl")
    # world_state.waiter_command_handler.go_to("ml_2")
    # world_state.waiter_command_handler.go_to("ml")
    # world_state.waiter_command_handler.go_to("tbl_bl")
    # world_state.waiter_command_handler.take_order(1)
    
    # while robot.step(timestep) != -1 and world_state.tables_states["tbl_bl"] != world_state.WAITING_FOR_FOOD:
    #     world_state.listen_to_entities(receiver)

    # world_state.waiter_command_handler.go_to("ml")
    # world_state.waiter_command_handler.go_to("ml_2")
    # world_state.waiter_command_handler.go_to("mr_1")
    # world_state.waiter_command_handler.go_to("tr_1")
    # world_state.waiter_command_handler.go_to("k")
    # world_state.waiter_command_handler.go_to("k_in")

    # # do a check for waiter in the kitchen
    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "k_in": # make check for food ready
    #     world_state.listen_to_entities(receiver)
    
    # world_state.waiter_command_handler.pick_up_food(1)

    # world_state.waiter_command_handler.go_to("k")
    # world_state.waiter_command_handler.go_to("tr_1")
    # world_state.waiter_command_handler.go_to("mr_1")
    # world_state.waiter_command_handler.go_to("ml_2")
    # world_state.waiter_command_handler.go_to("ml")
    # world_state.waiter_command_handler.go_to("tbl_bl")

    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "tbl_bl":
    #     world_state.listen_to_entities(receiver)

    # world_state.waiter_command_handler.deliver_food(1)
    # world_state.waiter_command_handler.go_to("ml")
    # world_state.waiter_command_handler.go_to("ml_1")
    
    # while robot.step(timestep) != -1 and world_state.waiter["position"] != "ml_1":
    #     world_state.listen_to_entities(receiver)

    # world_state.cleaner_command_handler.go_to("mr_1")
    # world_state.cleaner_command_handler.go_to("ml_2")
    # world_state.cleaner_command_handler.go_to("ml")
    # world_state.cleaner_command_handler.go_to("tbl_bl")
    # world_state.cleaner_command_handler.clean_table("tbl_bl")

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
    # explore_nodes_weights_with_dfs(graph, "tl_1", world_state.drone_command_handler.go_to, robot, timestep, world_state, receiver, world_state.drone)

    # explore_go_home_weights(graph, "tl_1", world_state.drone_command_handler.go_to, robot, timestep, world_state, receiver, world_state.drone)

    while robot.step(timestep) != -1 and len(world_state.groups_states) == 0:
        world_state.listen_to_entities(receiver)
    
    world_state.send_for_planning()

    while robot.step(timestep) != -1:
        world_state.listen_to_entities(receiver)
        world_state.follow_planning()

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
    
def explore_go_home_weights(graph, home_node_name, go_to_func, robot, timestep, world_state, receiver, entity):
    
    nodes_wieghts = dict()
    for node_name in graph.get_nodes():
        if node_name != home_node_name:
            go_to_func(node_name)
            while robot.step(timestep) != -1 and entity["position"] != node_name:
                world_state.listen_to_entities(receiver)
            start_time = robot.getTime()
            go_to_func(home_node_name)
            while robot.step(timestep) != -1 and entity["position"] != home_node_name:
                world_state.listen_to_entities(receiver)
            end_time = robot.getTime()
            weight = (end_time - start_time)
            nodes_wieghts[node_name] = weight
    
    for node, weight in nodes_wieghts.items():
        print(f'{node}: {weight}')

    return nodes_wieghts

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
