import numpy as np
from queue import PriorityQueue
from TrafficNode import *

# closed is set of already expaned nodes, dictionary from hstate to 
# node*hvalue*
def tree_search(frontier, heuristic, priority, closed, b=10):
    # initializations
#     _next = expand(start)
#     create the frontier list
#     frontier = PriorityQueue()

#     first visited node is the start node
#     start_node = PQNode(start,'',0)
#     frontier.push(start_node, 0)    
    # create a visited dictionary, separate from closed
    # visited only for this run of Astar, closed is for all runs thus far
    visited = {}

    # number of expansions
    N_exp = 0
    while (not frontier.empty()) and N_exp < b:
        (nodef, node) = frontier.get()
        hstate = node.getHashableState()
        # Add to visited list
        visited[hstate] = node
        if node.isGoal():
            print("FOUND GOAL")
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
            # if next_node.ancestors[-1] != node: 
            #     next_node.ancestors.append(node)
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
        

def astar_priority(g, h):
    return g + h

def traffic_heuristic(node):
    (gr, gc) = node.goal_position
    (br, bc) = node.bot_position
    return abs(gr - br) + abs(gc - bc)

def astar(start, frontier, heuristic, closed, b=10):
    return tree_search(start, frontier, heuristic, astar_priority, closed, b)




# safety_eval function from node to safety heuristic value
# priority takes h and g and returns f

def d_safe_search(start, C, safety_eval, priority, heuristic, b=10):
    if start.comfortable:
        b = 10
        f = priority(start.g, heuristic(start))
        C.put((f, start))
        return b, C
    
    d_safe_search_frontier = PriorityQueue()
    d_safe = safety_eval(start)
    d_safe_search_frontier.put((d_safe,start))
    
    N_exp = 0
    while N_exp < b and (not d_safe_search_frontier.empty()):
        node = d_safe_search_frontier.get()[1]
        _next = node.getAllSuccessors()
        N_exp += 1
        for action in _next.keys():
            # extract node that results from action
            next_node = _next.get(action)
            # complete its ancestor tree if it doesn't already know its immediate parent
            if next_node.ancestors[-1] != node: 
                next_node.ancestors.append(node)
                
            d_safe = safety_eval(next_node)
            d_safe_search_frontier.put((d_safe,next_node))
            if next_node.comfortable:
                for n in next_node.ancestors:
                    n.comfortable = True 
                b = 10
                f = priority(node.g, heuristic(node))
                C.put((f,node))
                return b, C
            
    return 2*b, C
    
def update(d, heuristic, frontier):
    R = PriorityQueue()
    #frontier list is list of frontier PNodes, contain parent field
    # newd maps things in d and frontier(d) to h values
    # d will just map things from d to h values
    newd = {}
    visited = set()
    # really just "possible direct parents" so cost is always 1, will still
    # propagate backwards to all ancestors
    ancestors = {}
    for k in d.keys():
        # set all h for s in I (initial list) to infinityish
        newd[k] = 100000
        R.put((newd[k], k))

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
            # h-value just original heuristic function, since not in d means
            # not updated, in d means already accounted for
            newd[hstate] = heuristic(node)
            R.put((newd[hstate], hstate))
            # only one level back
            ancestors[hstate] = [node.parent]
    # now r should contain I and frontier(I)
    while not R.empty():
        t_h, t = R.get()
        if(t_h == 100000):
            # all heuristic values should already have been reset, break
            break
        if(t in visited):
            continue
        visited.add(t)
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
                R.put((newval, k))
    # d should have updated values for the heuristics
    return d


def safeRTS(s_root, B, heuristic, priority, safety_eval):
    closed = {}
    while not s_root.isGoal():
        C = PriorityQueue()
        b = 10
        N_exp = 0
        found_s = False
        s_safe = None
        k = 0
        frontier = PriorityQueue()
        h = heuristic(s_root)
        f = priority(s_root.g, h)
        frontier.put((f, s_root))
        while N_exp < B:
            # perform A* for b expansions
            # frontier, closed = astar(s_root, s_goal, frontier, heuristic, b, k)
            frontier, closed = tree_search(frontier, heuristic, priority, closed, min(b, B-N_exp))

            N_exp += min(b, B-N_exp)
            if(N_exp == B):
                break
            # get the most "promising" node based on f value
            t_f, t = frontier.get()
            frontier.put((t_f,t)) # put it back in frontier
            # perform best-first search on d_safe starting from previously-determined promising node
            newb, C = d_safe_search(t, C, safety_eval, priority, heuristic, min(b, B-N_exp))
            N_exp += min(b, B-N_exp)
            b = newb
            
        # what's the set "open" ?
#         if not C.empty():
#             while not C.empty():
#                 f, s = C.get()
#                 num_safe = 0
#                 for ancestor in s.ancestors:
#                     if ancestor.isSafe():
#                         num_safe += 1
#                 if num_safe >= s_safe:
#                     found_s = True
#                     break

        # comfort has been propagated, so now search for the safe node 
        # predecessor of a best node
        while(not found_s and not frontier.empty()):
            _, best = frontier.get()
            # search predecessors from closest to farthest for a SAFE node
            if(best.isSafe()):
                found_s = True
                s_safe = best
            else:
                for i in range(len(best.ancestors)-1, -1, -1):
                    pred = best.ancestors[i]
                    if(pred.isSafe() and pred.g > s_root.g):
                        found_s = True
                        s_safe = pred
                        break


        if found_s:
            s_target = s_safe
        # check if identity action available (always rep by "_")
        elif s_root.apply("_") is not None:
            print("applying identity")
            s_target = s_root.apply("_")
        else:
            print("No safe path found")
            print("here is s_root tho")
            return s_root
        # update h values
        if(not frontier.empty()):
            closed = update(closed, heuristic, frontier)
        
        # set root equal to target (aka "move along path from root to target")
        s_root = s_target
        print("Current G is: "+str(s_root.g))
        # arbitrary cutoff, ideally shouldnt be neededd but /shrug
        if(s_root.g > s_root.height*s_root.width*2):
            print("TIMEOUT")
            return None
        # print(s_root.toString())
    if(s_root.isGoal()):
        return s_root
    else:
        return None

def traffic_safety_eval(node):
    # get min distance to a bunker
    min_dist = 10000
    for (r, c) in node.bunker_positions:
        dist = abs(r - node.bot_position[0]) + abs(c - node.bot_position[1])
        if(dist < min_dist):
            dist = min_dist
    return min_dist
succ_safe = 0
succ_lsslrta = 0
total_path_length_lss = 0
total_path_length_safe = 0
n = 15
for i in range(n):
    p = TrafficTest.generateStartNode(20, 20, 0.4, 0.1)
    from copy import deepcopy
    from lsslrta import lsslrta_other
    q = deepcopy(p)
    B = 100
    print(p.toString())
    r = safeRTS(p, B, traffic_heuristic, astar_priority, traffic_safety_eval)
    if(r is not None):
        print("SAFERTS")
        print(r.toString())
        print(len(r.path_actions))

        if(r.isGoal()):
            succ_safe+=1
            total_path_length_safe += len(r.path_actions)
   
        r2 = lsslrta_other(q, 10, traffic_heuristic, astar_priority)
        if(r2 is not None):
            print("LSSLRTA*")
            print(len(r2.path_actions))
            if(r2.isGoal()):
                succ_lsslrta += 1
                total_path_length_lss += len(r2.path_actions)

print("Total successful saferts: "+str(succ_safe))
print("Path length average: "+str(total_path_length_safe/n))
print("Total successful lsslrta: "+str(succ_lsslrta))
print("Path length average: "+str(total_path_length_lss/n))
