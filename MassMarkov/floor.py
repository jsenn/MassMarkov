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

    def add_agent(self, agent):
        self.population += 1

    def add_agents(self, agents):
        self.population += len(agents)

    def remove_agents(self, agents):
        self.population -= len(agents)
        assert(self.population >= 0)

