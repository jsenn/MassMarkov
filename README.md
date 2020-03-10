## TODO

Right now, this model predicts that people will move much more "efficiently" than does MassMotion when throttling/queueing effects come into play. For example, consider a situation where 2 5m-wide portals are separated by 8m on a single floor. Each portal births 1 agent every second for 100 seconds, for a total birth rate of 2 agents per second, and a total birth count of 200. What's happening is that the agents cross the floor, then have to wait to get on the portal to leave the simulation. This causes build-up at the portals, which affects the population of the floor greatly (maybe a factor of 2 or more). (TODO: this might not actually be the problem. Looking at the simulation closely, it looks like it may be more of an "internal friction" issue, which is harder to model...that said, the queueing effect does come into play at higher densities.)

Ideas for modelling this:
* Change speed/density relationship to be more local. Right now there's a single density for the entire route, the implicit assumption being that agents are evenly spread out along it. Of course, agent's *aren't* evenly spaced. This suggests using a more agent-based density measurement. Not sure how to do this: we could do it within the Route, but then we lose the ability for multiple Routes to share a floor. We could define the Routes as actual lines with start and end points, and do some basic line geometry to allow the Routes to talk to each other, but that's starting to get complicated.
* Each route has an "exit rate", or maybe a cap on the number of agents that can exit in a given frame. This could be purely geometric (i.e., width of exit space divided agent radius)
* Routes are connected by "Nodes", which have limited entry. Routes can't get rid of their agents without passing them to a Node.
  - I think this also suggests a nice architecture for the network: Nodes know how to marshal agents around. Any given Node may be connected to multiple Routes, and the Nodes assign agents to Routes. Routes are "dumb": they blindly accept agents from Nodes, and they can't get rid of agents on their own: they can only notify the Node on their "exit" side that some agents have traveled the entire length of the Route. Incidentally, this means that Routes have to be directed I think...
  - Rate-limiting could be implemented by attaching an intermediate Route/queue to the outgoing Node, which has a fixed capacity and a constant processing time

## Random ideas
* Use different models for velocity. Maybe 1D Social Forces or Moussaid's speed rule?
