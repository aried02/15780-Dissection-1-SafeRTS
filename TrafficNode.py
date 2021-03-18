import numpy as np
# double dictionary, first keyed by problem_id (different one for each 
# problem/run), then keyed by time step, as the car positions will always be
# the same for a given time step
car_cache = {}

def flip(orient):
    if(orient == "n"):
        return "s"
    elif(orient == "s"):
        return "n"
    elif(orient == "e"):
        return "w"
    else:
        return "e"

class TrafficNode:
    # up, left, down, right, do nothing
    possible_actions = ["u", "l", "d", "r", "_"]
    # north, south, east, west
    car_orientations = ["n", "s", "e", "w"]
    """
    Initializes new ProblemNode for Traffic problem
    in: cars: (int*int*char) list, denotes (r, c, orientation) r,c row col position of car (from top left 0,0), orientation (n/s/e/w)
    in: bunker_positions: (int*int) list, denotes (r,c) pos of bunkers
    in: bot_position: int*int, represents (r,c) position of robot/frog/thing we are moving
    in: goal_position: int*int, represnents (r,c) position of the goal
    in: path_actions: char list, actions to get to current node from start, [] if start
    in: problem_id: int, some unique id for this "problem" aka any nodes which
        are descended from the same start setup, for caching use
    in: g: int, length of path so far
    """
    def __init__(self, cars, bunker_positions, bot_position, goal_position, path_actions, height, width, problem_id, g=0):
        # sort these to ensure hashing returns same result always
        self.cars= sorted(cars)
        self.bunker_positions = bunker_positions
        self.bot_position = bot_position
        self.goal_position = goal_position
        self.path_actions = path_actions
        self.height = height
        self.width = width
        self.problem_id = problem_id
        self.g = g
        # initially only have comfortable true if safe
        self.comfortable = (bot_position in bunker_positions)
        if (problem_id not in car_cache):
            car_cache[problem_id] = {}
        # automatically caches on creation
        car_cache[problem_id][g] = cars
    
    def isGoal(self):
        return self.bot_position == self.goal_position
    
    def isSafe(self):
        return self.bot_position in self.bunker_positions
    
    # checks if this position collides with any cars, or is off the edge
    def collision(self, r, c, cars_in=None):
        if(cars_in is None):
            cars_in = self.cars
        if (r < 0 or c < 0 or r >= self.height or c >= self.width):
            return True
        for i in TrafficNode.car_orientations:
            if ((r,c,i) in cars_in):
                return True
        return False

    def isDead(self):
        (r, c) = self.bot_position
        return self.collision(r,c)
    
    # checks if equal to other traffic node from the same "run"
    def equals(self, t):
        # assuming the two are in the same run, we know that goal
        # position and bunker positions/height/width do not change
        return (self.bot_position == t.bot_position) and (self.cars == t.cars)

   

    # advances all cars one time step forward, bouncing if they are about to
    # enter a bunker position or hit the edge
    def advance(self):
        if(self.g+1 in car_cache[self.problem_id]):
            return car_cache[self.problem_id][self.g + 1]
        new_cars = []
        for (r, c, orient) in self.cars:
            (newr, newc, neworient) = (r,c,orient)
            if(orient == "n"):
                newr = r-1
            elif(orient == "s"):
                newr = r+1
            elif(orient == "e"):
                newc = c+1
            else:
                newc = c-1
            if(self.collision(newr, newc, new_cars) or (newr, newc) in self.bunker_positions or self.collision(newr, newc)):
                    # flip dir, dont move
                    neworient = flip(orient)
                    newr = r
                    newc = c
            # add new car to new_cars
            new_cars.append((newr,newc,neworient))
        # sort so same order through list every time
        return sorted(new_cars)
                

    # returns TrafficNode yielded by taking the given action, or None if the
    # trafficnode yielded is dead or impossible (i.e. moving up at the edge)
    def apply(self, action):
        if action not in TrafficNode.possible_actions:
            raise "Bad action passed to TrafficNode"
        # first advance all cars one time step
        new_cars = self.advance()
        # then take action
        (r, c) = self.bot_position
        (newr, newc) = (r, c)
        if(action == "u"):
            newr = r-1
        elif(action == "d"):
            newr = r+1
        elif(action == "l"):
            newc = c-1
        elif(action == "r"):
            newc = c+1
        # check to ensure we are not moving to a car space currently
        # (before we could "swap" with a car, which is impossible without
        # collision)
        new_node = TrafficNode(new_cars, self.bunker_positions, (newr, newc), self.goal_position, self.path_actions + [action], self.height, self.width, self.problem_id, self.g+1)

        if new_node.isDead() or self.collision(newr, newc):
            return None
        return new_node
    
    # returns dictionary of action to TrafficNode
    def getAllSuccessors(self):
        succs = {}
        for action in TrafficNode.possible_actions:
            res = self.apply(action)
            if(res is not None):
                succs[action] = res
        return succs

    def getAllPossiblePreviousStates(self):
        # returns all STATES - aka g*(bot_pos) - that could yield this node
        if(self.g == 0):
            return []
        oldg = self.g - 1
        # assuming we dont somehow go 2 time steps should be cached
        old_cars = car_cache[self.problem_id][oldg]
        (r, c) = self.bot_position
        bot_pos_changes = [(-1, 0), (0, -1), (1, 0), (0, 1), (0,0)]
        return [(oldg, (r+i[0], c+i[1])) for i in bot_pos_changes if 
                not self.collision(r+i[0], c+i[1], old_cars)]
    
    # needed for putting this into a priority queue (I think to break priority
    # based ties), paper says to break ties based on lower h, which means higher
    # g since priority = h+g
    def __lt__(self, othernode):
        return self.g > othernode.g
    
    # for caching assuming the same run, since cars at same time step will be 
    # the same, just need time elapsed (aka g) and bot_position
    def getHashableState(self):
        return (self.g, self.bot_position)
    
    def toString(self):
        # Just for easier visualization and testing
        grid = [["e" for i in range(self.width)] for j in range(self.height)]
        for (r,c, orient) in self.cars:
            if(orient == "n"):
                grid[r][c] = "^"
            elif(orient == "s"):
                grid[r][c] = "v"
            elif(orient == "e"):
                grid[r][c] = ">"
            else:
                grid[r][c] = "<"
        for (r,c) in self.bunker_positions:
            grid[r][c] = "b"
        (r,c) = self.bot_position
        grid[r][c] = "*"
        (r,c) = self.goal_position
        grid[r][c] = "G"
        s = ""
        for l in grid:
            for j in l:
                s += j + " "
            s += "\n"
        return s
# probabilities in range [0,1]
class TrafficTest:
    current_id = 0

    def generateStartNode(width, height, prob_car, prob_bunker):
        cars = []
        # even with this, could have gridlocked state
        bunkers = [(0,0)]
        goal_pos = (height-1, width-1)
        bot_pos = (0,0)

        for i in range(height):
            for j in range(width):
                if(i == 0 and j == 0):
                    continue
                if(np.random.random() <= prob_bunker):
                    bunkers.append((i,j))
                elif(np.random.random() <= prob_car):
                    a = np.random.random()
                    if (a <= 0.25):
                        orient = "n"
                    elif(a <= 0.5):
                        orient = "s"
                    elif(a <= 0.75):
                        orient = "e"
                    else:
                        orient = "w"
                    cars.append((i,j, orient))
        start_node = TrafficNode(cars, bunkers, bot_pos, goal_pos, [], height, width, TrafficTest.current_id)
        TrafficTest.current_id += 1
        return start_node

# # Test code to see cars/bot moving

# p = TrafficNode([(0,1,"e"), (2,2,"e"), (1,2,"n")], [(3,3), (2,3)], (0,0), (3,3), [], 4, 4, -1)
# q = TrafficTest.generateStartNode(10, 10, 0.35, 0.1)

# print(p.toString())
# d = p.getAllSuccessors()
# for a in d.keys():
#     print("Move "+a)
#     print(d[a].toString())
#     d2 = d[a].getAllPossiblePreviousStates()
#     print(d2)

# print(q.toString())
# d = q.getAllSuccessors()
# for a in d.keys():
#     print("Move "+a)
#     print(d[a].toString())
#     d2 = d[a].getAllPossiblePreviousStates()
#     print(d2)