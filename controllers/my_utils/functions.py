
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL
from classes_and_constans import CPU_CHANNEL, NODES_CHANNEL, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph
from classes_and_constans import get_graph

import math
import ast

channels_to_str = {CPU_CHANNEL: "CPU", NODES_CHANNEL: "NODES", WAITER_CHANNEL: "WAITER", CLEANER_CHANNEL: "CLEANER", DRONE_CHANNEL: "DRONE"}

def get_positions_graph_from_cpu(receiver, emitter, graph, got_positions = False):
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
                print(f"{channels_to_str[receiver.getChannel()]} Received position {name}: {position} from CPU")
                receiver.nextPacket()
    
    print(non_null_count, len_nodes)
    if non_null_count == len_nodes:
        print("All nodes position received")
        got_positions = True
        # send verification to cpu
        message = receiver.getChannel()
        emitter.setChannel(CPU_CHANNEL)
        emitter.send(str(message).encode('utf-8'))
    else:
        got_positions = False
    
    return got_positions