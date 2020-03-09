import math

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

    def clear_outgoing_agents(self):
        self.floor.remove_agents(self.outgoing_agents)
        self.outgoing_agents = []

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
    width = 2
    length = 8
    floor = Floor(width * length)
    route = FruinBiRoute(length, floor)

    dt = 0.2
    birth_rate = 2
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
        route.clear_outgoing_agents()

        floor_populations.append(float(floor.get_population()))

        time_since_last_birth += dt
        time += dt

    avg = mean(floor_populations)

    print("Simulation duration:\t%f" % time)
    print("Min population:\t%d" % min(floor_populations))
    print("Max population:\t%d" % max(floor_populations))
    print("Avg population:\t%f" % avg)
    print("Stddev population:\t%f" % stddev(floor_populations, avg))


