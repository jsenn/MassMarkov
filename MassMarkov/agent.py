import numpy as np


class Agent:
    _NEXT_ID = 0

    def __init__(self, preferred_speed_mps = 1.3, relaxation_time_s = 0.5):
        # static values
        self.id = Agent._NEXT_ID
        Agent._NEXT_ID += 1

        self.preferred_speed = preferred_speed_mps
        self.relaxation_time = relaxation_time_s

    @classmethod
    def get_rand(klass):
        speed = np.random.normal(loc=1.35, scale=0.3)
        speed = max(0, min(1.8, speed))
        return Agent(preferred_speed_mps=speed)


class AgentGenerator:
    def __iter__(self):
        return self

    def __next__(self):
        return self.get_next_agent()

    def get_next_agent(self):
        raise NotImplementedError()


class RandomAgentGenerator(AgentGenerator):
    def get_next_agent(self):
        return Agent.get_rand()
