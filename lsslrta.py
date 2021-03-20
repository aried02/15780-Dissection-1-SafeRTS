# Implementation of LSS-LRTA* from paper
from TrafficNode import *
from PriorityQueue import *
from queue import PriorityQueue

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
def update(d, heuristic, frontier):
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
    frontier_list = []
    while(not frontier.empty()):
        a = frontier.get()
        frontier_list.append(a)
    for i in range(len(frontier_list)):
        frontier.put(frontier_list[i])
        frontier_list[i] = frontier_list[i][1]
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
    pq.put((priority(root.g, heuristic(root)), root))
    # closed is map from hashable state to pnode*heuristic values * direct parents
    closed = {}
    while(not root.isGoal()):
        # Lookahead, perform A* expansion for "bound" rounds
        visited = {}
        for i in range(bound):
            # figure out most "promising" node based on priority
            if(pq.empty()):
                return None
            (blah, node) = pq.get()
            if(node.isGoal()):
                pq.put((blah, node))
                # stop just before expanding
                break
            hstate = node.getHashableState()
            visited[hstate] = 0
            if(hstate not in closed):
                closed[hstate] = (node, heuristic(node), [node.parent] if node.parent is not None else [])
            else:
                # update parents, keep heuristic val
                (nd, heur, p) = closed[hstate]
                newparents = p + [node.parent] if node.parent is not None else p
                closed[hstate] = (nd, heur, newparents)
            # push on all successors to open set
            for succ in node.getAllSuccessors().values():
                # dont revisit
                if(succ.getHashableState in visited):
                    continue
                # figure out heuristic value for this state
                (_, heur, __) = closed.get(succ.getHashableState(), (None, heuristic(succ), None))
                pq.put((priority(succ.g, heur), succ))
        # do update for all states in closed of heuristic value
        closed = update(closed, heuristic, pq)
        # find and set new root
        target = pq.get()[1]
        print(target.g)
        # arbitrary cutoff for now, should have loop avoidance later
        if(target.g > target.width*target.height*20):
            return None
        # once we commit to actions, we have to reset the priority queue and
        # root from here, since we cannot "go back" time steps
        root = target
        pq = PriorityQueue()
        (_, heur, __) = closed.get(target.getHashableState(), (None, heuristic(target), None)) 
        pq.put((priority(target.g, heuristic(target)), target))

    return root

def tree_search(start, frontier, heuristic, priority, closed, b=10):
    # initializations
#     _next = expand(start)
#     create the frontier list
#     frontier = PriorityQueue()

#     first visited node is the start node
#     start_node = PQNode(start,'',0)
#     frontier.push(start_node, 0)
    h = heuristic(start)
    f = priority(start.g, h)
#     k = 0
    frontier.put((f, start))
    
    # create a visited dictionary, separate from closed
    # visited only for this run of Astar, closed is for all runs thus far
    visited = {} 
    visited[start.getHashableState()] = 0 

    # number of expansions
    N_exp = 0
    
    while (not frontier.empty()) and N_exp < b:
        (nodef, node) = frontier.get()
        hstate = node.getHashableState()
        # Add to visited list
        visited[node.getHashableState()] = node
        if node.isGoal():
            frontier.put((nodef, node))
            return frontier, closed
        # Add to closed, with init hvalue, and list of direct parents for update
        if(hstate not in closed):
            closed[hstate] = (node, heuristic(node), [node.parent] if node.parent is not None else [])
        else:
            (nd, heur, p) = closed[hstate]
            newparents = p + [node.parent] if node.parent is not None else p
            closed[hstate] = (nd, heur, newparents)
        
        _next = node.getAllSuccessors() # should return a dictionary of resulting_nodes (all action costs are 1)
        N_exp += 1
        
        for action in _next.keys(): # action_space needs to be defined
            # extract node that results from action
            next_node = _next.get(action)
            # check if expanded node has already been visited
            if next_node.getHashableState() in visited:
                continue

            # complete its ancestor tree if it doesn't already know its immediate parent
            if next_node.ancestors[-1] != node: 
                next_node.ancestors.append(node)
            # # extract cost - cost automatically updated 
            # next_node.g = _next.get(action)[0] + node.g
            # # assign path that led to this state - already done
            # next_node.path = node.path+action
            # compute heuristic of this state
            if (next_node.getHashableState() in closed):
                h = closed[next_node.getHashableState()][1]
            else:
                h = heuristic(next_node)
            # compute f value
            f = priority(next_node.g, h)
            # add to the frontier
            frontier.put((f, next_node))
    # end of while
               
    # put the popped nodes back into the frontier 
    # - not sure why we would do this, so removed
    if(frontier.empty()):
        return None, closed     
    return frontier, closed

def lsslrta_other(root, bound, heuristic, priority):
    # maintains open list and closed list
    # open list is priority queue of nodes and f values
    # closed is a dictionary mapping hashable states to h values so easy to
    # update
    # closed is map from hashable state to pnode*heuristic values * direct parents
    closed = {}
    while(not root.isGoal()):
        frontier = PriorityQueue()
        # Lookahead, perform A* expansion for "bound" rounds
        frontier, closed = tree_search(root, frontier, heuristic, priority, closed, bound)
        if(frontier is None):
            return None
        # do update for all states in closed of heuristic value
        closed = update(closed, heuristic, frontier)
        # find and set new root
        target = frontier.get()[1]
        print("Current target time: "+str(target.g))
        # arbitrary cutoff for now, should have loop avoidance later
        if(target.g > target.width*target.height*10):
            return None
        # once we commit to actions, we have to reset the priority queue and
        # root from here, since we cannot "go back" time steps
        root = target

    return root
# # 0.5 causes slightly too many dead ends, sometimes loops - 
# # I think the infinite loops are caused by being forced into a position where
# # we can stay still in a bunker forever, but cannot find a way out
# p = TrafficTest.generateStartNode(20, 20, 0.4, 0.1)
# from copy import deepcopy
# q = deepcopy(p)
# print(p.toString())
# i = 1
# print("running with expansion budget of "+str(i*10))
# r2 = lsslrta_other(q, i*10, traffic_heuristic, astar_priority)
# if(r2 is None):
#     print("DEAD END")
# r = lsslrta(p, i*10, traffic_heuristic, astar_priority)
# if(r is None):
#     print("DEAD END 2 BOOGALOO")
# if(str(r.path_actions) != str(r2.path_actions)):
#     print("sad\n\n\n")
# print("First returned "+str(r.path_actions))
# print("First cost: "+str(len(r.path_actions)))
# print("Second returned: "+str(r2.path_actions))
# print("Second cost: "+str(len(r2.path_actions)))
# if(r is None):
#     print("DEAD END: GOT STUCK")
# print(len(r.path_actions))
# # q = p
# # import time
# # print("Initial")
# # print(q.toString())
# # for action in r.path_actions:
# #     q = q.apply(action)
# #     print(action)
# #     print(q.toString())
# #     time.sleep(0.5)



