
class TrafficNode:
    # up, left, down, right, do nothing
    possible_actions = ["u", "l", "d", "r", "_"]
    car_orientations = ["n", "s", "e", "w"]
    """
    Initializes new ProblemNode for Traffic problem
    in: cars: (int*int*char) list, denotes (r, c, orientation) r,c row col position of car (from top left 0,0), orientation (n/s/e/w)
    in: bunker_positions: (int*int) list, denotes (r,c) pos of bunkers
    in: bot_position: int*int, represents (r,c) position of robot/frog/thing we are moving
    in: goal_position: int*int, represnents (r,c) position of the goal
    in: ancestors: TrafficNode list, ancestors of current node, [] if start
    in: g: int, length of path so far
    """
    def __init__(self, cars, bunker_positions, bot_position, goal_position, ancestors, height, width, g=0):
        self.cars= cars
        self.bunker_positions = bunker_positions
        self.bot_position = bot_position
        self.goal_position = goal_position
        self.ancestors = ancestors
        self.height = height
        self.width = width
        self.g = g
        # initially only have comfortable true if safe
        self.comfortable = (bot_position in bunker_positions)
    
    def isGoal(self):
        return self.bot_position == self.goal_position
    
    def isSafe(self):
        return self.bot_position in self.bunker_positions
    
    # checks if this position collides with any cars
    def carCollision(cars_in, r, c):
        for i in TrafficNode.car_orientations:
            if ((r,c,i) in cars_in):
                return True
        return False

    def isDead(self):
        (r, c) = self.bot_position
        outofbounds = (r < 0 or c < 0 or r >= self.height or c >= self.width)
        return TrafficNode.carCollision(self.cars,r,c) or outofbounds
    
    # checks if equal to other traffic node from the same "run"
    def equals(self, t):
        # assuming the two are in the same run, we know that goal
        # position and bunker positions/height/width do not change
        return (self.bot_position == t.bot_position) and (self.cars == t.cars)

   

    # advances all cars one time step forward, bouncing if they are about to
    # enter a bunker position or hit the edge
    def advance(self):
        new_cars = []
        for (r, c, orient) in self.cars:
            (newr, newc, neworient) = (r,c,orient)
            if(orient == "n"):
                newr = r-1
                if(newr < 0 or TrafficNode.carCollision(new_cars, newr, newc) or (newr, newc) in self.bunker_positions):
                    # flip dir, dont move
                    neworient = "s"
                    newr = r
            elif(orient == "s"):
                newr = r+1
                if(newr >= self.height or TrafficNode.carCollision(new_cars, newr, newc) or (newr, newc) in self.bunker_positions):
                    newr = r
                    neworient = "n"
            elif(orient == "e"):
                newc = c+1
                if(newc >= self.width or TrafficNode.carCollision(new_cars, newr, newc) or (newr, newc) in self.bunker_positions):
                    newc = c
                    neworient = "w"
            else:
                newc = c-1
                if(newc < 0 or TrafficNode.carCollision(new_cars, newr, newc) or (newr, newc) in self.bunker_positions):
                    newc = c
                    neworient = "e"
            # add new car to new_cars
            new_cars.append((newr,newc,neworient))
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

        new_node = TrafficNode(new_cars, self.bunker_positions, (newr, newc), self.goal_position, self.ancestors + [self], self.height, self.width, self.g+1)

        if new_node.isDead():
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


"""
# Test code to see cars/bot moving

p = TrafficNode([(0,1,"e"), (2,2,"e"), (1,2,"n")], [(3,3), (2,3)], (0,0), (3,3), [], 4, 4)

print(p.toString())
d = p.getAllSuccessors()
for a in d.keys():
    print("Move "+a)
    print(d[a].toString())
    d2 = d[a].getAllSuccessors()
    for b in d2.keys():
        print("Move "+a+ " "+b)
        print(d2[b].toString())

"""