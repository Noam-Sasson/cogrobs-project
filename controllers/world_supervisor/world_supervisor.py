from controller import Supervisor
import numpy as np

import sys
import os
import random
import ast

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constans import CPU_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, DRONE_CHANNEL, RATE_OF_ARRIVAL, FOOD_ITEMS, RATE_OF_PREP_TIME, DISH_OCUNT

CUST_NUM = 8
init_xyloc = np.array([-2.5, -6])
offsets = [np.array([0, 0]), np.array([0, -0.5]), np.array([-0.5, 0]), np.array([-0.5, -0.5])]

GROUPS = {f'g{i}':{"members":[f'g{i}_p1', f'g{i}_p2', f'g{i}_p3', f'g{i}_p4'], 
                   "locs_outside": [init_xyloc + offset - np.array([0,i-1]) for offset in offsets]} for i in range(1, CUST_NUM+1)}


class WorldGenerator(Supervisor):
    def __init__(self, poisson_rate):
        super(WorldGenerator, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.current_time = 0
        self.poisson_rate = poisson_rate
        self.next_group_time = self.sample_exponential(self.poisson_rate)
        self.next_group_num = 1
        self.called_all_groups = False
        self.emitter = self.getDevice('emitter')
        self.foods_to_be_made = dict()
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.time_step)
        self.dishes_status = {f"dish_{i}": None for i in range(1, DISH_OCUNT + 1)}

    def sample_exponential(self, rate):
        """Sample from an exponential distribution with the given rate."""
        return np.random.exponential(1/rate)
    
    
    
    def get_node_by_name(self, name):
        """Get a node by its DEF name."""
        return self.getFromDef(name)

    def call_group_by_poisson(self):
        if self.current_time >= self.next_group_time:
            # Call a group of pedestrians
            if self.next_group_num == len(GROUPS) + 1:
                if not self.called_all_groups:
                    print("Called all groups")
                    self.called_all_groups = True
                return
            
            group = f'g{self.next_group_num}'
            group_size = random.choice([2, 3, 4])
            self.next_group_num += 1
            print(f"Calling group {group}")
            for pedestrian in GROUPS[group]["members"][:group_size]:
                node = self.get_node_by_name(pedestrian)
                if node:
                    position = node.getField("translation").getSFVec3f()
                    # print(f"Node {pedestrian} position: {position}")
                else:
                    print(f"Node {pedestrian} not found")
            # Sample the next event time
            self.next_group_time += self.sample_exponential(self.poisson_rate)

            self.make_pedestrians_arrive(group, group_size)

            self.emitter.setChannel(CPU_CHANNEL)
            message = (WORLD_GENERATOR_CHANNEL, "group_arrived", group, group_size)
            self.emitter.send(str(message).encode('utf-8'))

    def run(self):
        # Main loop:
        while self.step(self.time_step) != -1:
            # # Example usage: Get a node by name and print its position
            # for group, pedestrians in GROUPS.items():
            #     for pedestrian in pedestrians:
            #         node = self.get_node_by_name(pedestrian)
            #         if node:
            #             position = node.getField("translation").getSFVec3f()
            #             print(f"Node {pedestrian} position: {position}")
            #         else:
            #             print(f"Node {pedestrian} not found")
            self.current_time += self.time_step
            self.listen_to_cpu()
            self.call_group_by_poisson()
            self.make_food()

    def make_pedestrians_arrive(self, group, group_size):
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        for pedestrian in GROUPS[group]["members"][:group_size]:
            ped_idx =  GROUPS[group]["members"].index(pedestrian)
            location = GROUPS[group]["locs_outside"][ped_idx]
            message = (WORLD_GENERATOR_CHANNEL, pedestrian, "go_to", group_size, location[0], location[1])
            self.emitter.send(str(message).encode('utf-8'))

    def make_food(self):
        groups_to_delete = []
        for group, food in self.foods_to_be_made.items():
            # print(self.current_time, food["time"])
            if food["time"] <= self.current_time:
                self.emitter.setChannel(CPU_CHANNEL)
                message = (WORLD_GENERATOR_CHANNEL, "food_ready", group, food["dish"])
                self.emitter.send(str(message).encode('utf-8'))
                groups_to_delete.append(group)
                self.dishes_status[food["dish"]] = None
                print(f"Kitchen: Food ready for group {group}")

        for group in groups_to_delete:
            del self.foods_to_be_made[group]
    
    def listen_to_cpu(self): # check if there is a message from the CPU
        if self.receiver.getQueueLength() > 0:
            message = ast.literal_eval(self.receiver.getString())
            if message[0] == CPU_CHANNEL:
                if message[1] == "make_food":
                    group = message[2]
                    dish = message[3]
                    food = message[4:]
                    next_food_time = self.sample_exponential(RATE_OF_PREP_TIME)
                    print(f"Kitchen: Received food to be made: {food} for group {group} on dish {dish} time {next_food_time/1000}")
                    self.foods_to_be_made[group] = {"items": food, "time": self.current_time + next_food_time, "dish": dish}
                    if self.dishes_status[dish] is not None:
                        print(f"WARNING: Kitchen: Dish {dish} is already in use by group {self.dishes_status[dish]}")
                        
                    self.dishes_status[dish] = group
                    print(f"Kitchen: Food to be made: {food} at time {next_food_time} on dish {dish}")

            self.receiver.nextPacket()

if __name__ == "__main__":
    generator = WorldGenerator(poisson_rate = RATE_OF_ARRIVAL)
    generator.run()