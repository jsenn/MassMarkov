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


if __name__ == "__main__":
    width = 5
    length = 8
    floor = Floor(width * length)
    routeLR = MoussaidRoute(length, floor)
    routeRL = MoussaidRoute(length, floor)
    sourceL = ConstantRateSource(100, 1, 1)
    sourceR = ConstantRateSource(100, 1, 1)
    sinkL = Sink()
    sinkR = Sink()

    network = Network()
    network.add_node(sourceL)
    network.add_node(sourceR)
    network.add_node(sinkL)
    network.add_node(sinkR)
    network.add_node(routeLR)
    network.add_node(routeRL)

    network.connect_nodes(sourceL, routeLR)
    network.connect_nodes(routeLR, sinkR)
    network.connect_nodes(sourceR, routeRL)
    network.connect_nodes(routeRL, sinkL)

    dt = 0.2
    birth_rate = 1
    assert(birth_rate > 0)
    time_between_births = 1.0 / birth_rate

    floor_populations = []

    time = 0
    max_time = 200
    while time < max_time:
        network.update(dt)
        floor_populations.append(floor.get_population())
        time += dt

    avg = mean(floor_populations)

    print("Simulation duration:\t%f" % time)
    print("Min population:\t%d" % min(floor_populations))
    print("Max population:\t%d" % max(floor_populations))
    print("Avg population:\t%f" % avg)
    print("Stddev population:\t%f" % stddev(floor_populations, avg))
    print("\n".join(map(str, floor_populations)))

