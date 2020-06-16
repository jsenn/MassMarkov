class Node:
    def __init__(self):
        # dynamic values
        self.inbox = [] # [Agent]
        self.outbox = [] # [Agent]

    def update(self, dt):
        pass

    def add_outgoing_agents(self, agents):
        self.outbox += agents

    def add_outgoing_agent(self, agent):
        self.outbox.append(agent)

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

    def peek_incoming_agents(self):
        return self.inbox[:]

    def pull_n_agents(self, n):
        assert(n <= len(self.outbox))
        agents = self.outbox[:n]
        self.outbox = self.outbox[n:]
        return agents

    def push_agents(self, agents):
        assert(len(agents) <= self.in_capacity())
        self.inbox += agents

    def clear_incoming_agents(self):
        self.inbox = []

class Sink(Node):
    def __init__(self):
        super().__init__()

    def add_agents_to_outbox(self, agents):
        raise RuntimeError("Can't produce agents from a Sink Node!")

    def can_connect_in(self, other): return True
    def can_connect_out(self, other): return False
    def is_root(self): return False

