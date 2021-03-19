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

def astar_priority(g, h):
    return g+h

# update step of lsslrta*, takes in dictionary of hashableState -> (PNode, h)
# returns dictionary with updated h values

# I think this is right, not extremely efficient but not terrible
def update(d, heuristic, frontier_list):
    R = {}
    #frontier list is list of frontier PNodes, contain parent field
    # newd maps things in d and frontier(d) to h values
    # d will just map things from d to h values
    newd = {}
    # really just "possible direct parents" so cost is always 1, will still
    # propagate backwards to all ancestors
    ancestors = {}
    for k in d.keys():
        # set all h for s in I (initial list) to infinityish
        (pnode, _, __) = d[k]
        R[k] = 0
        newd[k] = 100000

        # #find frontier nodes via successors of these and push actual h values
        # for i in pnode.getAllSuccessors().values():
        #     hstate = i.getHashableState()
        #     if(hstate in d):
        #         continue
        #     newd[hstate] = heuristic(i)
        #     if(hstate not in ancestors):
        #         ancestors[hstate] = []
        #     ancestors[hstate].append(pnode)
        #     R[hstate] = 0
    for node in frontier_list:
        # want to find heuristic of these if not in closed already
        hstate = node.getHashableState()
        if(hstate not in d):
            R[hstate] = 0
            # h-value just original heuristic function, since not in d means
            # not updated, in d means already accounted for
            newd[hstate] = heuristic(node)
            # only one level back
            ancestors[hstate] = [node.parent]
    # now r should contain I and frontier(I)
    while len(R.keys()) != 0:
        t = min(R.keys(), key=(lambda k: newd[k]))
        t_h = newd[t]
        R.pop(t)
        if(t in ancestors):
            t_ancestors = ancestors[t]
        else:
            t_ancestors = d[t][2]
        for pnode in t_ancestors:
            # update h value, cost = 1 direct parent
            newval = 1 + t_h
            k = pnode.getHashableState()
            if(newd[k] > newval):
                if(k in d):
                    d[k] = (d[k][0], newval, d[k][2])
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
    pq.push(root, priority(root.g, heuristic(root)))
    # closed is map from hashable state to pnode*heuristic values * direct parents
    closed = {}
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
            hstate = node.getHashableState()
            if(hstate not in closed):
                closed[hstate] = (node, heuristic(node), [node.parent] if node.parent is not None else [])
            else:
                # update parents, keep heuristic val
                (nd, heur, p) = closed[hstate]
                newparents = p + [node.parent] if node.parent is not None else p
                closed[hstate] = (nd, heur, p+[node.parent])
            # push on all successors to open set
            for succ in node.getAllSuccessors().values():
                # figure out heuristic value for this state
                (_, heur, __) = closed.get(succ.getHashableState(), (None, heuristic(succ), None))
                pq.push(succ, priority(succ.g, heur))
        # do update for all states in closed of heuristic value
        closed = update(closed, heuristic, map(lambda x: x[1], pq.elements))
        # find and set new root
        target = pq.pop()
        print(target.g)
        # arbitrary cutoff for now, should have loop avoidance later
        if(target.g > target.width*target.height*20):
            return None
        # once we commit to actions, we have to reset the priority queue and
        # root from here, since we cannot "go back" time steps
        root = target
        pq = PriorityQueue()
        (_, heur, __) = closed.get(target.getHashableState(), (None, heuristic(target), None)) 
        pq.push(target, priority(target.g, heuristic(target)))

    return root


# 0.5 causes slightly too many dead ends, sometimes loops - 
# I think the infinite loops are caused by being forced into a position where
# we can stay still in a bunker forever, but cannot find a way out
p = TrafficTest.generateStartNode(20, 20, 0.4, 0.1)
print(p.toString())
i = 1
print("running with expansion budget of "+str(i*10))
r = lsslrta(p, i*10, traffic_heuristic, astar_priority)
if(r is None):
    print("DEAD END: GOT STUCK")
print(len(r.path_actions))
q = p
import time
print("Initial")
print(q.toString())
for action in r.path_actions:
    q = q.apply(action)
    print(action)
    print(q.toString())
    time.sleep(0.5)



