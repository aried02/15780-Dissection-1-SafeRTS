# 15780-Dissection-1-SafeRTS
Implementation of SafeRTS for AI paper dissection


Interface for Problems, idea is to represent them as individual nodes with functions to get next states, check current state is goal/safe/etc, and so forth. Should hopefully be abstracted so that the SafeRTS implementation can run with either of the problems with no issue.
```
type ProblemNode (PNode) :
  type action; Represents all actions that can be taken at a given time
  public var comfortable: bool; can be used to cache comfort of this node when found comfortable in safeRTS
  public var ancestors: PNode list; represents all ancestors of the current node, set on creation
  public var costFromStart: int; keeps track of cost of path so far for ASTAR, set on creation
  isGoal: self -> bool; Checks if current node is a goal state
  isSafe: self -> bool; Checks if current node is safe
  isDead: self -> bool; Checks if current node is a deadend (i.e. collision in traffic/must run into wall in racetrack)
  equals: self * PNode -> bool; given another PNode, says whether the two are equal
  apply: self * action -> PNode; Gives the resulting node after an action is taken and one time step elapses
  getAllSuccessors: self -> (action: PNode) dictionary; dictionary keyed by possible actions and itemized by resulting state
``` 
  
