import math

from collections import dequeue

# A = 281 feet per minute
# B = 752 foot-modules per second (feet per minute * square feet per person)
A_uni = 1.43
B_uni = 0.354

# A = 267 feet per minute
# B = 722 foot-modules per second
A_bi = 1.36
B_bi = 0.341


class Agent:
    _NEXT_ID = 0

    def __init__(self):
        self.id = Agent._NEXT_ID
        Agent._NEXT_ID += 1


class Floor:
    def __init__(self, area):
        # static values
        self.area = float(area)

        # dynamic values
        self.population = 0

    def get_population(self):
        return self.population

    def get_density(self):
        return self.population / self.area

    def get_agent_module(self):
        return self.area / self.population if self.population > 0 else self.area

    def add_agents(self, agents):
        self.population += len(agents)

    def remove_agents(self, agents):
        self.population -= len(agents)
        assert(self.population >= 0)


class Route:
    def __init__(self, distance, floor):
        # static values
        self.distance = float(distance)
        self.floor = floor

        # dynamic values
        self.agents = []
        self.incoming_agents = []
        self.outgoing_agents = []
        self.agent_id_to_progress = {}

    def get_agent_speed(self, agent):
        raise NotImplementedError()

    def set_incoming_agents(self, agents):
        self.incoming_agents = agents
        self.floor.add_agents(agents)

    def get_outgoing_agents(self):
        return self.outgoing_agents

    def clear_outgoing_agents(self, count):
        done_agents = self.outgoing_agents[:count]
        self.floor.remove_agents(done_agents)
        self.outgoing_agents = self.outgoing_agents[count:]

    def update(self, dt):
        for agent in self.incoming_agents:
            self.agents.append(agent)
            self.agent_id_to_progress[agent.id] = 0
        self.incoming_agents = []

        remaining_agents = []
        for agent in self.agents:
            self.agent_id_to_progress[agent.id] += self.get_agent_speed(agent) * dt
            if self.agent_id_to_progress[agent.id] >= self.distance:
                self.outgoing_agents.append(agent)
                del self.agent_id_to_progress[agent.id]
            else:
                remaining_agents.append(agent)
        self.agents = remaining_agents


class FruinUniRoute(Route):
    def __init__(self, distance, floor):
        super().__init__(distance, floor)

    def get_agent_speed(self, agent):
        M = self.floor.get_agent_module()
        assert(M > 0)
        return A_uni - B_uni / M


class FruinBiRoute(Route):
    def __init__(self, distance, floor):
        super().__init__(distance, floor)

    def get_agent_speed(self, agent):
        M = self.floor.get_agent_module()
        assert(M > 0)
        return A_bi - B_bi / M


class ConstantSpeedRoute(Route):
    def __init__(self, speed, distance, floor):
        super().__init__(distance, floor)

        # static values
        self.speed = speed

    def get_agent_speed(self, agent):
        return self.speed


class ConstantTimeRoute(Route):
    def __init__(self, time, distance, floor):
        super().__init__(distance, floor)

        #static values
        self.crossing_time = float(time)

    def get_agent_speed(self, agent):
        return self.get_distance() / self.crossing_time


class Source:
    def __init__(self, agent_count):
        # static values
        self.total_agent_count = agent_count

        # dynamic values
        self.agents_left = self.total_agent_count
        self.outgoing_agents = [] # [Agent]

    def get_agents_left(self): return self.agents_left

    def produce_agents(self, agents):
        self.outgoing_agents += agents
        self.agents_left -= len(agents)

    def take_all(self):
        return self.take_agents(len(self.outgoing_agents))

    def take_up_to_n(self, count):
        agents_taken = self.outgoing_agents[:count]
        self.outgoing_agents = self.outgoing_agents[count:]
        return agents_taken

    def update(self, dt):
        raise NotImplementedException()



class Sink:
    def __init__(self):
        pass

    def update(self, dt):
        pass


class Sink:
    def __init__(self):
        super().__init__()

    def add_agents_to_outbox(self, agents):
        raise RuntimeError("Can't produce agents from a Sink Node!")

    def can_connect_in(self, other): return True
    def can_connect_out(self, other): return False
    def is_root(self): return False

class Node:
    def __init__(self):
        # dynamic values
        self.inbox = [] # [Agent]
        self.outbox = [] # [Agent]

    def update(self, dt):
        pass

    def add_agents_to_outbox(self, agents):
        self.outbox += agents

    def can_connect_in(self, other):
        raise NotImplementedError()

    def can_connect_out(self, other):
        raise NotImplementedError()

    def is_root(self):
        raise NotImplementedError()

    def in_capacity(self):
        return float("inf")

    def peek_outgoing_agents(self):
        return self.outbox[:]

    def pull_n_agents(self, n):
        assert(n <= len(self.outbox))
        agents = self.outbox[:n]
        self.outbox = self.outbox[n:]
        return agents

    def push_agents(self, agents):
        assert(len(agents) <= self.in_capacity())
        self.inbox += agents


class Source(Node):
    def __init__(self, max_agents_produced=float("inf")):
        super().__init__(self)
        # static values
        self.max_agents_produced = max_agents_produced
        # dynamic values
        self.agents_left = max_agents_produced

    def get_agents_left(self):
        return self.agents_left

    def in_capacity(self): return 0
    def can_connect_in(self): return False
    def can_connect_out(self): return True

    def add_agents_to_outbox(self, agents):
        assert(len(agents) <= self.agents_left)
        super().add_agents_to_outbox(agents)
        self.agents_left -= len(agents)


class ConstantRateSource(Source):
    def __init__(self, agent_count, agent_chunk_size, time_between_chunks):
        super().__init__(agent_count)
        # static values
        self.chunk_size = agent_chunk_size
        self.time_between_chunks = float(time_between_chunks)
        # dynamic values
        self.time_since_last_chunk = 0.0

    def update(self, dt):
        self.time_since_last_chunk += dt
        while self.get_agents_left() > 0 and self.time_since_last_chunk > self.time_between_chunks:
            agents_to_produce = min(self.get_agents_left(), self.chunk_size)
            self.add_agents_to_outbox([Agent() for _ in range(agents_to_produce)])
            self.time_since_last_chunk -= self.time_between_chunks

class Network:
    def __init__(self):
        # dynamic values
        self.nodes = []
        self.root_indexes = [] # [int]
        self.node_to_children = {} # {Node: set([Node])}
        self.node_to_parents = {} # {Node: set([Node])}

    def has_node(self, node):
        # TODO: hacky
        return node in self.node_to_children

    def add_node(self, node):
        if self.has_node(node):
            return

        self.nodes.append(node)
        if node.is_root():
            self.root_indexes.append(len(self.nodes) - 1)

        self.node_to_children[node] = set([])
        self.node_to_parents[node] = set([])

    def connect_nodes(self, nodeA, nodeB):
        assert(self.has_node(nodeA) and self.has_node(nodeB))
        assert(nodeA.can_connect_out(nodeB))
        assert(nodeB.can_connect_in(nodeA))

        children = self.node_to_children[nodeA]
        assert(nodeB not in children)
        children.add(nodeB)

        parents = self.node_to_parents[nodeB]
        assert(nodeA not in parents)
        parents.add(nodeA)

    def update(self, dt):
        for node in self.nodes:
            node.update(dt)
        self.compute_costs()
        self.propagate_agents()

    def compute_costs(self):
        pass

    def propagate_agents(self):
        q = dequeue([])
        for idx in self.root_indexes:
            q.append(self.nodes[idx])

        while len(q) > 0:
            curr = q.popleft()
            parents = self.node_to_parents[curr]
            if len(parents) == 0:
                break;
            capacity = curr.in_capacity()
            incoming_queues = []
            total_incoming_agents = 0
            for parent in parents:
                incoming = dequeue(parent.peek_outgoing_agents())
                if len(incoming) > 0:
                    incoming_queues.append(incoming)
                    total_incoming_agents += len(incoming)

            # sort the queues by length, longest to shortest
            sort(incoming_queues, key=lambda xs: len(xs), reverse=True)

            # distribute agents to the current node in a round-robin fashion:
            num_agents_left = min(capacity, total_incoming_agents)
            agents_taken_per_parent = [[] for parent in parents]
            incoming_idx = 0
            while num_agents_left > 0:
                curr_queue = incoming_queues[incoming_idx]
                agents_taken = agents_taken_per_parent[incoming_idx]
                incoming_idx = (incoming_idx + 1) % len(incoming_queues)
                if len(curr_queue) == 0:
                    continue

                agents_taken.append(curr_queue.popleft())
                num_agents_left -= 1

            for parent_idx in range(len(parents)):
                parent = parents[parent_idx]
                transferred_agents = agents_taken_per_parent[parent_idx]
                parent.pull_n_agents(len(transferred_agents))
                curr.push_agents(transferred_agents)

def mean(xs):
    assert(len(xs) > 0)
    return sum(xs) / float(len(xs))

def stddev(xs, avg):
    assert(len(xs) >= 2)
    val = 0
    for x in xs:
        diff = x - avg
        val += diff * diff

    return math.sqrt(val / (len(xs) - 1))

if __name__ == "__main__":
    width = 5
    length = 8
    floor = Floor(width * length)
    routeLR = FruinBiRoute(length, floor)
    routeRL = FruinBiRoute(length, floor)
    sourceL = ConstantRateSource(100, 1, 1);
    sinkL = Sink()

    dt = 0.2
    birth_rate = 1
    assert(birth_rate > 0)
    time_between_births = 1.0 / birth_rate

    floor_populations = []

    time = 0
    time_since_last_birth = 0
    agents_left = 200
    while agents_left > 0 or floor.get_population() > 0:
        new_agents = []
        if agents_left > 0:
            while time_since_last_birth >= time_between_births:
                new_agents.append(Agent())
                time_since_last_birth -= time_between_births
                agents_left -= 1

        route.set_incoming_agents(new_agents)
        route.update(dt)

        done_agents = route.get_outgoing_agents()
        route.clear_outgoing_agents(len(done_agents))

        floor_populations.append(float(floor.get_population()))

        time_since_last_birth += dt
        time += dt

    avg = mean(floor_populations)

    print("Simulation duration:\t%f" % time)
    print("Min population:\t%d" % min(floor_populations))
    print("Max population:\t%d" % max(floor_populations))
    print("Avg population:\t%f" % avg)
    print("Stddev population:\t%f" % stddev(floor_populations, avg))
    print("\n".join(map(str, floor_populations)))

