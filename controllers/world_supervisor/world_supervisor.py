from controller import Supervisor
import numpy as np

import sys
import os
import random

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constans import CPU_CHANNEL, WORLD_GENERATOR_CHANNEL, PEDESTRIAN_CHANNEL, DRONE_CHANNEL

init_xyloc = np.array([-2.5, -6])
offsets = [np.array([0, 0]), np.array([0, -0.5]), np.array([-0.5, 0]), np.array([-0.5, -0.5])]

GROUPS = {f'g{i}':{"members":[f'g{i}_p1', f'g{i}_p2', f'g{i}_p3', f'g{i}_p4'], 
                   "locs_outside": [init_xyloc + offset - np.array([0,i-1]) for offset in offsets]} for i in range(1, 9)}


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
                    print(f"Node {pedestrian} position: {position}")
                else:
                    print(f"Node {pedestrian} not found")
            # Sample the next event time
            self.next_group_time += self.sample_exponential(self.poisson_rate)
            self.emitter.setChannel(CPU_CHANNEL)
            message = (WORLD_GENERATOR_CHANNEL, "group_arrived", group, group_size)
            self.emitter.send(str(message).encode('utf-8'))
            self.make_pedestrians_arrive(group, group_size)

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
            self.call_group_by_poisson()

    def make_pedestrians_arrive(self, group, group_size):
        self.emitter.setChannel(PEDESTRIAN_CHANNEL)
        for pedestrian in GROUPS[group]["members"][:group_size]:
            ped_idx =  GROUPS[group]["members"].index(pedestrian)
            location = GROUPS[group]["locs_outside"][ped_idx]
            message = (WORLD_GENERATOR_CHANNEL, pedestrian, "go_to", location[0], location[1])
            self.emitter.send(str(message).encode('utf-8'))                   

if __name__ == "__main__":
    avrg_time_of_arrival = 0.1*60*1000 # 2 minutes
    generator = WorldGenerator(poisson_rate = 1/avrg_time_of_arrival)
    generator.run()