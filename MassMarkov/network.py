from collections import deque

from node import Node


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
        self._compute_costs()
        self._propagate_agents()

    def _compute_costs(self):
        pass

    def _propagate_agents(self):
        q = deque([])
        for idx in self.root_indexes:
            q.append(self.nodes[idx])

        while len(q) > 0:
            curr = q.popleft()
            for child in self.node_to_children[curr]:
                q.append(child)

            parents = list(self.node_to_parents[curr])
            if len(parents) == 0:
                continue;
            capacity = curr.in_capacity()
            incoming_queues = []
            total_incoming_agents = 0
            for parent in parents:
                incoming = deque(parent.peek_outgoing_agents())
                if len(incoming) > 0:
                    incoming_queues.append(incoming)
                    total_incoming_agents += len(incoming)

            # sort the queues by length, longest to shortest
            incoming_queues.sort(key=lambda xs: len(xs), reverse=True)

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

