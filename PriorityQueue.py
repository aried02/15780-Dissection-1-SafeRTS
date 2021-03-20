from heapq import *

class PriorityQueueFake:

    def __init__(self):
        self.elements = []

    def nonempty(self):
        return bool(self.elements)

    def push(self, element, priority):
        heappush(self.elements, (priority, element))

    def pop(self):
        return heappop(self.elements)[1]

    def peek(self):
        return self.elements[0][1]
    #checks if pq contains ProblemNode
    def contains(self, node):
        return any(
            node.equals(element)
            for priority, element in self.elements
        )