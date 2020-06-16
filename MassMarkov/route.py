from floor import Floor
from node import Node


# A = 281 feet per minute
# B = 752 foot-modules per second (feet per minute * square feet per person)
A_uni = 1.43
B_uni = 0.354

# A = 267 feet per minute
# B = 722 foot-modules per second
A_bi = 1.36
B_bi = 0.341


class Route(Node):
    def __init__(self, distance, floor):
        super().__init__()

        # static values
        self.distance = float(distance)
        self.floor = floor

        # dynamic values
        self.agents = []
        self.agent_id_to_progress = {}

    def get_agent_speed(self, agent):
        raise NotImplementedError()

    def get_concentration(self):
        return len(self.agents) / self.distance

    def get_distance_to_next_agent(self, agent):
        agent_progress = self.agent_id_to_progress[agent.id]
        min_greater_progress = float("inf")
        for id in self.agent_id_to_progress:
            curr_progress = self.agent_id_to_progress[id]
            if curr_progress <= agent_progress or id == agent.id:
                continue
            if curr_progress < min_greater_progress:
                min_greater_progress = curr_progress
        return min_greater_progress

    def is_root(self):
        return False

    def can_connect_in(self, other): return True
    def can_connect_out(self, other): return True

    def add_agent(self, agent):
        self.agents.append(agent)
        self.agent_id_to_progress[agent.id] = 0
        self.floor.add_agent(agent)

    def update(self, dt):
        for agent in self.peek_incoming_agents():
            self.add_agent(agent)
        self.clear_incoming_agents()

        remaining_agents = []
        removed_agents = []
        for agent in self.agents:
            self.agent_id_to_progress[agent.id] += self.get_agent_speed(agent) * dt
            if self.agent_id_to_progress[agent.id] >= self.distance:
                removed_agents.append(agent)
            else:
                remaining_agents.append(agent)
        self.add_outgoing_agents(removed_agents)
        self.floor.remove_agents(removed_agents)
        for agent in removed_agents:
            del self.agent_id_to_progress[agent.id]
        self.agents = remaining_agents


class NonInteractingRoute(Route):
    def __init__(self, distance, floor):
        super().__init__(distance, floor)

    def get_agent_speed(self, agent):
        return agent.preferred_speed


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


class MoussaidRoute(Route):
    def __init__(self, distance, floor):
        super().__init__(distance, floor)

    def get_agent_speed(self, agent):
        distance_to_next_in_front = self.get_distance_to_next_agent(agent)
        max_safe_speed = distance_to_next_in_front / agent.relaxation_time
        v_des = min(agent.preferred_speed, max_safe_speed)
        # TODO: keep track of speeds, and set dv/dt = (v_des - v_curr) / tau?
        return v_des

