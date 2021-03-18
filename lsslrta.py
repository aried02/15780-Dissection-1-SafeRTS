# Implementation of LSS-LRTA* from paper
from TrafficNode import *
from PriorityQueue import *

# random example
p = TrafficNode([(0,1,"w"), (2,2,"e"), (1,2,"n")], [(3,3), (2,3)], (0,0), (3,3), [], 4, 4, -1)

# simple manhattan distance heuristic plus g so far
def traffic_heuristic(node):
    (gr, gc) = node.goal_position
    (br, bc) = node.bot_position
    return abs(gr - br) + abs(gc - bc)

def traffic_priority(node):
    return traffic_heuristic(node)+node.g

# update step of lsslrta*, takes in dictionary of hashableState -> (PNode, h)
# returns dictionary with updated h values

# For our purposes I do not think this step is necessary, we will never 
# encounter an unexpected obstacle since we can model the space exactly

def update(d, heuristic):
    R = {}
    # newd maps things in d and frontier(d) to h values
    # d will just map things from d to h values
    newd = {}
    for k in d.keys():
        # set all h for s in I (initial list) to infinityish
        (pnode, _) = d[k]
        R[k] = 0
        newd[k] = 100000
        #find frontier nodes via successors of these and push actual h values
        for i in pnode.getAllSuccessors().values():
            newd[i.getHashableState()] = heuristic(i)
            R[i.getHashableState()] = 0
    # now r should contain I and frontier(I)
    while len(R) != 0:
        t = min(R, key=(lambda k: newd[k]))
        t_h = newd[t]
        R.pop(t)
        # wildly inefficient :(, no way currently to get all ancestor states
        # shouldnt be too difficult, but some car collisions may be nondet
        # since based on ordering in grid
        print(d)
        for k in d.keys():
            print(type(d))
            (pnode,_) = d[k]
            succs = [i.getHashableState() for i in pnode.getAllSuccessors().values()]
            if(t in succs):
                # update h value, cost = 1 direct parent
                newval = 1 + t_h
                if(newd[k] > newval):
                    d[k] = (d[k][0], newval)
                    newd[k] = newval
    # d should have updated values for the heuristics
    return d

# print(update({p.getHashableState(): (p, 0)}, traffic_heuristic))

def lsslrta(root, bound, heuristic, priority):
    # maintains open list and closed list
    # open list is priority queue of nodes and f values
    # closed is a dictionary mapping hashable states to h values so easy to
    # update
    pq = PriorityQueue()
    pq.push(root, priority(root))
    while(not root.isGoal()):
        # Lookahead, perform A* expansion for "bound" rounds
        for i in range(bound):
            # figure out most "promising" node based on priority
            if(not pq.nonempty()):
                return None
            node = pq.peek()
            if(node.isGoal()):
                # stop just before expanding
                break
            node = pq.pop()
            # push on all successors to open set
            for succ in node.getAllSuccessors().values():
                pq.push(succ, priority(succ))
        # here we would update, but nah
        target = pq.pop()
        print("Updating root, time step now: "+str(target.g))
        # once we commit to actions, we have to reset the priority queue and
        # root from here, since we cannot "go back" time steps
        root = target
        pq = PriorityQueue()
        pq.push(target, priority(target))

    return root


# 0.5 causes slightly too many dead ends, sometimes loops - 
# I think the infinite loops are caused by being forced into a position where
# we can stay still in a bunker forever, but cannot find a way out
p = TrafficTest.generateStartNode(10, 10, 0.4, 0.1)
r = lsslrta(p, 10, traffic_heuristic, traffic_priority)
if(r is None):
    print("DEAD END: GOT STUCK")
q = p
print("Initial")
print(q.toString())
for action in r.path_actions:
    q = q.apply(action)
    print(action)
    print(q.toString())



