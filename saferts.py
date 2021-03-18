import numpy as np
from queue import PriorityQueue

def tree_search(start, is_goal, frontier, expand, action_space, heuristic, priority, b=10, k=0):
    # initializations
#     _next = expand(start)
#     create the frontier list
#     frontier = PriorityQueue()

#     first visited node is the start node
#     start_node = PQNode(start,'',0)
#     frontier.push(start_node, 0)
    h = heuristic(start)
    priority = h + start.g
#     k = 0
    frontier.put((priority, k, start))
    
    #     create a visited dictionary
    visited = {} 
    visited[start] = 0 

    # number of expansions
    N_exp = 0
    
    while (not frontier.empty()) and N_exp < b:
        node = frontier.get()[2]
        # Add to visited list
        visited[node.state] = node
        if is_goal(node.state):
            return list(str(node.path))
        else:
            _next = expand(node) # expand ideally should return a dictionary of (cost_of_action, resulting_node)
            N_exp += 1
            
            for action in action_space: # action_space needs to be defined
                # extract node that results from action
                next_node = _next.get(action)[1]
#                 # check if expanded node has already been visited
                if next_state in visited.keys():
                    continue

                # complete its ancestor tree if it doesn't already know its immediate parent
                if next_node.ancestors(end) != node: 
                    next_node.ancestors.append(node)
                # extract cost
                next_node.g = _next.get(action)[0] + node.g
                # assign path that led to this state
                next_node.path = node.path+action
                # compute heuristic of this state
                next_node.h = heuristic(next_node)
                # compute f value
                next_node.f = priority(g, h)
                # add to the frontier
                k += 1
                frontier.put((next_node.f, k, next_node))
    # end of while
               
# put the popped nodes back into the frontier
    for state in visited.keys():
        k += 1
        frontier.put((visited[state].f, k, visited[state]))
                              
    return frontier, k
        

def astar_priority(g, h):
    return g + h

def astar(start, is_goal, frontier, expand, action_space, heuristic, b=10, k=0):
    return tree_search(start, is_goal, frontier, expand, action_space, heuristic, astar_priority, b, k)





def d_safe_search(start, C, expand, action_space, safety_eval, b=10):
    if start.isSafe() or start.isComfortable():
        b = 10
        C.put((start.f, start))
        return b, C
    
    d_safe_search_frontier = PriorityQueue()
    d_safe = safety_eval(start)
    d_safe_search_frontier.put((d_safe,k,start))
    
    N_exp = 0
    while N_exp < b and (not d_safe_search_frontier.empty()):
        node = d_safe_search_frontier.get()[2]
        _next = expand(node)
        N_exp += 1
        for action in action_space:
            # extract node that results from action
            next_node = _next.get(action)[1]
            next_node.path = node.path+action
            next_node.g = node.g + _next.get(action)[0]
            # complete its ancestor tree if it doesn't already know its immediate parent
            if next_node.ancestors(end) != node: 
                next_node.ancestors.append(node)
                
            d_safe = safety_eval(next_node)
            d_safe_search_frontier.put((d_safe,next_node))
            if next_node.isComfortable():
                for n in next_node.ancestors:
                    n.comfortable = True 
                b = 10
                C.put((node.f,node))
                return b, C
            
    return 2*b, C
    



def safeRTS(s_root, s_goal, B):
    while s_root != s_goal:
        C = PriorityQueue()
        b = 10
        N_exp = 0
#         s_safe = 1 what's s_safe?
        found_s = False
        
        k = 0
        frontier = PriorityQueue()
        while N_exp < B:
            # perform A* for b expansions
            frontier, k = astar(s_root, s_goal, frontier, expand, action_space, heuristic, b, k)
            # get the most "promising" node based on f value
            t_f, t_k, t = frontier.get()
            frontier.put((t_f,t_k,t)) # put it back in frontier
            # perform best-first search on d_safe starting from previously-determined promising node
            b, C = d_safe_search(t, C, expand, action_space, safety_eval, b)
            
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
        if found_s:
            s_target = s_safe
        elif s_root.identityAction:
            s_target = s_root
        else:
            raise TypeError("no safe path is available")
                        
