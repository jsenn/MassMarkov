## TODO

Right now, this model predicts that people will move much more "efficiently" than does MassMotion when throttling/queueing effects come into play. For example, consider a situation where 2 5m-wide portals are separated by 8m on a single floor. Each portal births 1 agent every second for 100 seconds, for a total birth rate of 2 agents per second, and a total birth count of 200. What's happening is that the agents cross the floor, then have to wait to get on the portal to leave the simulation. This causes build-up at the portals, which affects the population of the floor greatly (maybe a factor of 2 or more).

Ideas for modelling this:
* Each route has an "exit rate", or maybe a cap on the number of agents that can exit in a given frame. This could be purely geometric (i.e., width of exit space divided agent radius)
* Routes are connected by "Nodes", which have limited entry. Routes can't get rid of their agents without passing them to a Node.
  - I think this also suggests a nice architecture for the network: Nodes know how to marshal agents around. Any given Node may be connected to multiple Routes, and the Nodes assign agents to Routes. Routes are "dumb": they blindly accept agents from Nodes, and they can't get rid of agents on their own: they can only notify the Node on their "exit" side that some agents have traveled the entire length of the Route. Incidentally, this means that Routes have to be directed I think...
