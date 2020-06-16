import itertools

from agent import Agent
from node import Node


class Source(Node):
    def __init__(self, agent_generator, max_agents_produced=float("inf")):
        super().__init__()
        # static values
        self.agent_generator = agent_generator
        self.max_agents_produced = max_agents_produced
        # dynamic values
        self.agents_left = max_agents_produced

    def is_root(self):
        return True

    def get_agents_left(self):
        return self.agents_left

    def in_capacity(self): return 0
    def can_connect_in(self, other): return False
    def can_connect_out(self, other): return True

    def add_agents_to_outbox(self, agents):
        assert(len(agents) <= self.agents_left)
        super().add_outgoing_agents(agents)
        self.agents_left -= len(agents)


class ConstantRateSource(Source):
    def __init__(self, agent_generator, agent_count, agent_chunk_size, time_between_chunks):
        super().__init__(agent_generator, agent_count)
        # static values
        self.chunk_size = agent_chunk_size
        self.time_between_chunks = float(time_between_chunks)
        # dynamic values
        self.time_since_last_chunk = 0.0

    def update(self, dt):
        self.time_since_last_chunk += dt
        while self.get_agents_left() > 0 and self.time_since_last_chunk > self.time_between_chunks:
            agents_to_produce = min(self.get_agents_left(), self.chunk_size)
            self.add_agents_to_outbox([agent for agent in itertools.islice(self.agent_generator, agents_to_produce)])
            self.time_since_last_chunk -= self.time_between_chunks

