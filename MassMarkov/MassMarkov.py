import math
import numpy as np
import matplotlib.pyplot as plt

from agent import *
from floor import *
from route import *
from source import *
from node import *
from network import *
from util import *


def do_sim(length, birth_num, birth_time, max_time, interaction_dist=2, dt=0.2):
    width = 5
    floor = Floor(width * length)
    route = NonInteractingRoute(length, floor)
    source = ConstantRateSource(RandomAgentGenerator(), float("inf"), birth_num, birth_time)
    sink = Sink()

    network = Network()
    network.add_node(source)
    network.add_node(sink)
    network.add_node(route)

    network.connect_nodes(source, route)
    network.connect_nodes(route, sink)

    floor_concentrations = []
    interactions = {}

    def record_interaction(idA, idB, dt):
        min_id = min(idA, idB)
        max_id = max(idA, idB)
        if min_id not in interactions:
            interactions[min_id] = {}
        if max_id not in interactions[min_id]:
            interactions[min_id][max_id] = 0
        interactions[min_id][max_id] += dt

    time = 0
    while time < max_time:
        network.update(dt)
        floor_concentrations.append(route.get_concentration())
        time += dt

        # record interactions
        agent_progress_pairs = []
        for agent in route.agents:
            agent_progress_pairs.append((agent.id, route.agent_id_to_progress[agent.id]))
        agent_progress_pairs.sort(key=lambda pair: pair[1])

        for i in range(len(agent_progress_pairs)):
            id, progress = agent_progress_pairs[i]

            # record all neighbours to the right
            for j in range(i, len(agent_progress_pairs)):
                other_id, other_progress = agent_progress_pairs[j]
                if other_id == id:
                    continue
                progress_diff = abs(other_progress - progress)
                if progress_diff > interaction_dist:
                    break
                record_interaction(id, other_id, dt)
            # record all neighbours to the left
            for j in range(i-1, -1, -1):
                other_id, other_progress = agent_progress_pairs[j]
                if other_id == id:
                    continue
                progress_diff = abs(other_progress - progress)
                if progress_diff > interaction_dist:
                    break
                record_interaction(id, other_id, dt)

    return interactions, floor_concentrations

if __name__ == "__main__":
    # Facts about avg. degree so far:
    # -------------------------------
    # * reverse sigmoid-ish curve for increasing time cutoff
    # * proportional to length
    # * proportional to birth num
    def get_deg_dist(interactions, time_cutoff):
        hist = []
        for id in interactions:
            deg = 0
            for other_id in interactions[id]:
                if interactions[id][other_id] >= time_cutoff:
                    deg += 1
            while len(hist) <= deg:
                hist.append(0)
            hist[deg] += 1
        return hist

    def get_avg_deg(hist):
        total_sum = 0
        weight_sum = 0
        for deg in range(len(hist)):
            weight = hist[deg]
            total_sum += weight * deg
            weight_sum += weight
        return total_sum / weight_sum

    birth_time = 1
    length = 20
    max_time = 400
    birth_nums = [i for i in range(1, 11)]
    avg_degs = []
    for birth_num in birth_nums:
        interactions, floor_concentrations = do_sim(length, birth_num, birth_time, max_time)

        avg_concentration = mean(floor_concentrations)

        print("Simulation duration:\t%f" % max_time)
        print("Min concentration:\t%f" % min(floor_concentrations))
        print("Max concentration:\t%f" % max(floor_concentrations))
        print("Avg concentration:\t%f" % avg_concentration)
        print("Stddev concentration:\t%f" % stddev(floor_concentrations, avg_concentration))

        avg_degs.append(get_avg_deg(get_deg_dist(interactions, 0)))

    plt.plot(birth_nums, avg_degs)
    plt.show()
