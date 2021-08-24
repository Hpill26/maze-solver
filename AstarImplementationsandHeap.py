# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 11:36:38 2020

@author: Harry
"""
import heapq
import numpy as np
import random as rr
import IPython
class Node:
    def __init__(self, parent=None, pos=None):
        self.parent = parent
        self.pos = pos

        self.f = 0
        self.manhattan = 0
        self.cost = 0

    def __eq__(self, other):
        return self.pos == other.pos
    
    #for heap
    def __lt__(self, other):
      return self.f < other.f or self.cost > other.cost
    
  
    def __gt__(self, other):
      return self.f > other.f
class doubleLL:
    def __init__(self):
        self.front = None
        self.size = 0
    def insert(self, newPos):
        #first node to be added to the list
        if self.front is None:
            newNode = Node();
            newNode.pos = newPos;
            print("newNode was ", newNode.pos)
            print("startNode parent is?", newNode.parent)
            self.front = newNode;
            return
            #self.size += 1;
        else:
            temp = self.front;
            while temp.next is not None:
                #print("temp before adding prev is", temp.parent)
                #prev = temp;
                #print("prev node: ",prev.pos);
                temp = temp.next;
            newNode = Node();
            newNode.pos = newPos;
            #print("newNode's parent is ", newNode.parent.pos)
            #print("newNode is ", newNode.pos)
            temp.next = newNode;
            newNode.parent = temp;
            #self.size += 1;
    def searchDLL(self,searchNode):
        retVal = 0;
        temp = self.front;
        while temp is not None:
            if(temp == searchNode):
                retVal = 1;
            temp = temp.next;
        return retVal;
    def printDLL(self,goalNode):
        path = [] 
        temp = goalNode;
        #path.append(temp.pos);
        print("path appended: ", temp.pos)
        while temp is not None:
            path.append(temp.pos);
            print("path appended: ", temp.pos)
            temp = temp.parent;
            #print("temp's parent is ", temp.parent.pos)
            #print("temp is ", temp.pos)
            #print("temp is ", temp.pos)
            
        return path;
class singleLL:
    def __init__(self):
        self.front = None;
    def insert(self, newNode):
        if self.front == None:
            newNode.next = None;
            self.front = newNode;
        else:
            newNode.next = self.front;
    def printLL(self,frontNode):
        while frontNode is not None:
            print(frontNode)
            frontNode = frontNode.next;
class priorotyQueueNodes:
    def __init__(self):
        self.list = []
        #self.Node = Node;
        self.size = 0
    #inserts desired Node into priority queue and decides value by F value
    def insert(self,insertNode):
        self.list.append(Node(insertNode,insertNode.pos))
        self.size += 1
        #self.list[self.size].Node = Node;
        
        j = self.size
        #print("nodes size", self.size)
        while j // 2 > 0:
           # print("j = ", j)
            #checking f value to properly add nodes to priority queue
            k = self.list[j // 2]
           # print("k = ", k.pos)
            if self.list[j-1].f < k.f:
                self.list[j // 2] = self.list[j-1]
                self.list[j-1] = k
            #if we have a tied f value then check the g vlaue
            elif self.list[j-1].f == k.f:
                #let larger g value have higher priority, so switch if k has bigger g
                if k.cost > self.list[j-1].cost:
                     self.list[j // 2] = self.list[j-1]
                     self.list[j-1] = k
            j //= 2
        
    def delete(self):
        retVal = self.list[0]
        if self.size != 1:
            self.list[0] = self.list[self.size];
            #print("delete has been entered")
        self.size -= 1
        i = 1
        while (i * 2) <= self.size:
            lastChild = self.Child(i)
            #switch position if current node has bigger f or has same f but smaller g(ie tie break)
            if self.list[i].f > self.list[lastChild].f or ((self.list[i].f == self.list[lastChild].f) and (self.list[i].cost < self.list[lastChild].cost)):
                temp = self.list[i]
                self.list[i] = self.list[lastChild]
                self.list[lastChild] = temp
            i = lastChild
        
        self.insert(retVal);
        return retVal
    
    def searchQ(self, searchNode):
        temp = self.list;
        i = 0;
        ret = 0;
        if self.size == 0:
            return 0
        while(i < self.size):
            if(temp[i] == searchNode):
                #print("cost of the node is ", searchNode.cost)
                return searchNode.cost;
            else:
                ret = -1
            i += 1;
            
        return ret;
    #hopefully i can return this stupid pos shit cause im losing my patience
    def Child(self,i):
        if i * 2 + 1 > self.size:
            return i * 2
        else:
            if self.list[i*2].f >= self.list[i*2+1].f:
                return i * 2 + 1
            else:
                return i * 2
    
    def printHeap(self):
        i = 1;
        while(i <= self.size):
            if self.size == 0:
                print("empty list")
            else:
                print(self.list[i])
            #print("i inside printHeap is " + str(i))
            i += 1
def repeatedBackward(maze, start, end):
    rows,cols = np.shape(maze)
    
    #create empty maze
    emptyMaze = [[0 for i in range(rows)] for j in range(cols // 2)]
    #print("size of empty maze " , len(emptyMaze))
    #print(emptyMaze)
    
   #create robot with vision and update maze
    agent = start
    #moves available to us
    North = (1, 0)
    South = (0, 1)
    East = (-1, 0)
    West = (0, -1)
    
    actions = (South,North,East,West)
    
    if maze[start[0]][start[1]] == 1:
        return "Path could not be found 1 "
    #generating moves so we can initially update the maze before I call A star for the first time
    
    for move in actions:
        neighbor = (agent[0] + move[0], agent[1] + move[1])
        
        #within range?
        if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
            continue
        
        #now that we know that its within range time to update neighbors
        if maze[neighbor[0]][neighbor[1]] == 1:
            emptyMaze[neighbor[0]][neighbor[1]] = 1
            #print(emptyMaze)
    #implement same counter here its taking way too long
    outer_iterations = 0
    max_iterations = (len(maze[0]) ** 2)
    #now that it is initialized call A* on it and find the agent path
    #iterate until agent is at destination
    #initialize the agents moved block's list this is what I will use to find the path the agent took
    movedBlocks = []
    #problem here is that this shit won't work if start is also the end
    if(start == end):
        movedBlocks.append(agent);
    while agent != end:
        agentPath = []
        currentNode = Astar(emptyMaze,end, agent)
        
        if(outer_iterations == max_iterations):
            return "Path could not be found 2"
        outer_iterations += 1
        #print("a star has returned: ", agentPath[0])
        if agentPath is None:
            return "Path could not be found 3"
        
        curr = currentNode
        while curr is not None:
            agentPath.append(curr.pos)
            curr = curr.parent
        agentPath = agentPath[::-1]
        
        #added the node the agent is at to the moved block now moving to next step
        
        #remove first item since we added it to the moved Blocks list
        #problem here 
        #movedBlocks.append(agent);
        #print("mover Block is ", movedBlocks[0])
        #iterate thru the path aStar returned
        i = 0
        numOFiters = 0;
        #iterator variable to iterate thru the agentPath list
        while i < len(agentPath):
            
            agent = agentPath[i]
            #print("agent is at ", agent)
            #generating next moves for agent and checking if we encounter a 1
            for move in actions:
                neighbor = (agent[0] + move[0], agent[1] + move[1])
        
                #within range?
                if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
                    continue
        
                #now that we know that its within range time to update neighbors
                if maze[neighbor[0]][neighbor[1]] == 1:
                    emptyMaze[neighbor[0]][neighbor[1]] = 1
                    #print(emptyMaze)
            
            
            #now to check if we can keep following this path agentPath's next item shall be our next move so 
            #check i + 1
            if(i+1) != len(agentPath):
                nextItem = agentPath[i+1]
                #print("next item is ", nextItem)
                if emptyMaze[nextItem[0]][nextItem[1]] == 1:
                    #print("empty maze value: ", emptyMaze[nextItem[0]][nextItem[1]])
                    #stop the loop
                    i = len(agentPath)
            #since the agent path isn't blocked we can keep going
            movedBlocks.append(agent);
            agentPath.remove(agent);
            
            numOFiters += 1
            #print("num of iterations: ",numOFiters)
        #print("exited i loop")
    return movedBlocks
def repeatedForward(maze, start, end):
    rows,cols = np.shape(maze)
    
    if maze[start[0]][start[1]] == 1:
        return "Path could not be found"
    
    #print("create empty maze")
    emptyMaze = [[0 for i in range(rows)] for j in range(cols)]
   # print("size of empty maze " , len(emptyMaze))
   # print(emptyMaze)
    
    #create robot with vision and update maze
    agent = start
    #moves available to us
    North = (1, 0)
    South = (0, 1)
    East = (-1, 0)
    West = (0, -1)
    
    actions = (South,North,East,West)
    
    #generating moves so we can initially update the maze before I call A star for the first time
    
    for move in actions:
        neighbor = (agent[0] + move[0], agent[1] + move[1])
        
        #within range?
        if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
            continue
        
        #now that we know that its within range time to update neighbors
        if maze[neighbor[0]][neighbor[1]] == 1:
            emptyMaze[neighbor[0]][neighbor[1]] = 1
            #print(emptyMaze)
    
    #now that it is initialized call A* on it and find the agent path
    #iterate until agent is at destination
    #initialize the agents moved block's list this is what I will use to find the path the agent took
    movedBlocks = []
    #problem here is that this shit won't work if start is also the end
    if(start == end):
        movedBlocks.append(agent);
    aStarCounter = 1;
    while agent != end:
        agentPath = []
        currentNode = Astar(emptyMaze,agent,end)
        
        
        agentPath = []
        curr = currentNode
        while curr is not None:
            agentPath.append(curr.pos)
            curr = curr.parent
              
        
        agentPath = agentPath[::-1]
        
        if agentPath is None:
            return "Path could not be found"
        print("a star has returned: ", agentPath[0])
        
        #added the node the agent is at to the moved block now moving to next step
        if emptyMaze[start[0]][start[1]] == 1:
            return "Path could not be found"
        agentPath.remove(agent); #remove first item since we added it to the moved Blocks list
        #problem here 
        #movedBlocks.append(agent);
        #print("mover Block is ", movedBlocks[0])
        #iterate thru the path aStar returned
        
        
        print("Iterating thru the returned A star path for the", str(aStarCounter)," time...\n")
        aStarCounter += 1;
        i = 0
        numOFiters = 0;
        #iterator variable to iterate thru the agentPath list
        while i < len(agentPath):
            
            agent = agentPath[i]
            print("agent is at ", agent)
            #generating next moves for agent and checking if we encounter a 1
            for move in actions:
                neighbor = (agent[0] + move[0], agent[1] + move[1])
        
                #within range?
                if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
                    continue
        
                #now that we know that its within range time to update neighbors
                if maze[neighbor[0]][neighbor[1]] == 1:
                    emptyMaze[neighbor[0]][neighbor[1]] = 1
                    #print(emptyMaze)
            
            
            #now to check if we can keep following this path agentPath's next item shall be our next move so 
            #check i + 1
            if(i+1) != len(agentPath):
                nextItem = agentPath[i+1]
                #print("next item is ", nextItem)
                if emptyMaze[nextItem[0]][nextItem[1]] == 1:
                    #print("empty maze value: ", emptyMaze[nextItem[0]][nextItem[1]])
                    #stop the loop
                    i = len(agentPath)
            #since the agent path isn't blocked we can keep going
            movedBlocks.append(agent);
            agentPath.remove(agent);
            
            numOFiters += 1
            #print("num of iterations: ",numOFiters)
        #print("exited i loop")
    return movedBlocks
def generatePossibleMoveset(actions, maze , current_node):
    
    possibleMoves = []
    for move in actions: # Adjacent squares

            # Get node position
            currentState = (current_node.pos[0] + move[0], current_node.pos[1] + move[1])
            
            #print("checking boundaries of maze")
            if  currentState[1] > (len(maze[len(maze)-1]) -1) or currentState[0] > (len(maze) - 1) or currentState[1] < 0 or currentState[0] < 0:
                continue
            #print("boundaries were checked""
            
            #print("here in moves")
            if maze[currentState[0]][currentState[1]] != 0:
                continue
            #print("moves wasn't skipped by blocked cell check")

            new_node = Node(current_node, currentState)

            # Append
            possibleMoves.append(new_node)
    return possibleMoves
def determineValidity(possibleMoves, expanded, visited, current_node, end):
    for move in possibleMoves:
            
            #print("checking if move was already expanded")
            if len([possibleMove for possibleMove in expanded if possibleMove == move]) > 0:
                continue
            #print("wasn't skipped")
            
            # Calculate and store the cost, manhattan distance and f values
            move.cost = current_node.cost + 1
            move.manhattan = calculateManhattan(move, end)
            move.f = calculateF(move, end)

            #print("checking if in visited list")
            possibleMove = 0
            if len([node for node in visited if move.pos == node.pos and move.cost > move.cost]) > 0:
                continue
            #print("not in visited list")
            
            heapq.heappush(visited, move)
    return visited
def determineValidity2(possibleMoves, expanded, visited, current_node, end, adaptive, updateCost,iterativeNode):
    for move in possibleMoves:
            #print("checking if move was already expanded")
            if len([possibleMove for possibleMove in expanded if possibleMove == move]) > 0:
                continue
            #print("wasn't skipped")

            # Calculate and store the cost, manhattan distance and f values
            move.cost = current_node.cost + 1
            move.manhattan = calculateManhattan(move, end)
            if adaptive == 1:
                move.f = updateF(move, end, iterativeNode, updateCost);
            else:
                move.f = calculateF(move, end)

            #print("checking if in visited list")
            possibleMove = 0
            if len([possibleMove for possibleMove in visited if move.pos == possibleMove.pos]) > 0:
                if(move.cost > possibleMove.cost):
                    continue
            #print("not in visited list")

            # Add the child to the open list
            heapq.heappush(visited, move)
    return visited
#adaptive a-Star updates heuristic after first run
def adaptive(maze, start, end):
    counter2 = 0
    rows,cols = np.shape(maze)
    
    if maze[start[0]][start[1]] == 1:
        return "Path could not be found"
    #create empty maze
    emptyMaze = [[0 for i in range(rows)] for j in range(cols)]
    #print("size of empty maze " , len(emptyMaze))
    #print(emptyMaze)
    
    #create robot with vision and update maze
    agent = start
    #moves available to us
    North = (1, 0)
    South = (0, 1)
    East = (-1, 0)
    West = (0, -1)
    
    actions = (South,North,East,West)
    
    
     #generating moves so we can initially update the maze before I call A star for the first time
    
    for move in actions:
        neighbor = (agent[0] + move[0], agent[1] + move[1])
        
        #within range?
        if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
            continue
        
        #now that we know that its within range time to update neighbors
        if maze[neighbor[0]][neighbor[1]] == 1:
            emptyMaze[neighbor[0]][neighbor[1]] = 1
            #print(emptyMaze)
    
    #now that it is initialized call A* on it and find the agent path
    #iterate until agent is at destination
    #initialize the agents moved block's list this is what I will use to find the path the agent took
    movedBlocks = []
    #problem here is that this shit won't work if start is also the end
    if(start == end):
        movedBlocks.append(agent);
        
    agentPath = []
    currentNode = Astar(emptyMaze,agent,end)
        
    agentPath = []
    curr = currentNode
    while curr is not None:
        agentPath.append(curr.pos)
        curr = curr.parent
              
        
    agentPath = agentPath[::-1]
        
    if agentPath is None:
        return ("Path could not be found")
        #print("a star has returned: ", agentPath[0])
        
        #added the node the agent is at to the moved block now moving to next step
    if emptyMaze[start[0]][start[1]] == 1:
        return "Path could not be found"
    
    
    #initalize that variable to be our new start 
    newStart = ();
    i = 0
    numOFiters = 0;
        #iterator variable to iterate thru the agentPath list
    while i < len(agentPath):
            
        agent = agentPath[i]
        #print("agent is at ", agent)
        #generating next moves for agent and checking if we encounter a 1
        for move in actions:
            neighbor = (agent[0] + move[0], agent[1] + move[1])
        
                #within range?
            if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
                    continue
        
        #now that we know that its within range time to update neighbors
            if maze[neighbor[0]][neighbor[1]] == 1:
                emptyMaze[neighbor[0]][neighbor[1]] = 1
                #print(emptyMaze)
            
            
        #now to check if we can keep following this path agentPath's next item shall be our next move so 
        #check i + 1
        if(i+1) != len(agentPath):
            nextItem = agentPath[i+1]
            #print("next item is ", nextItem)
            if emptyMaze[nextItem[0]][nextItem[1]] == 1:
                #print("empty maze value: ", emptyMaze[nextItem[0]][nextItem[1]])
                #stop the loop
                newStart = agent
                i = len(agentPath)
        #since the agent path isn't blocked we can keep going
        movedBlocks.append(agent);
        agentPath.remove(agent);
            
        numOFiters += 1
    #say we encounter a block this is where we should update the nodes and call our new function
    iterativeNode = currentNode;
    updateCost = 0
    #initialize vars so we can update the cost and f values accordingly
    while iterativeNode is not None:
        if newStart == currentNode.pos:
            updateCost = currentNode.cost
            break
        else:
            iterativeNode = iterativeNode.parent
            
    # now that i found the update value time to call A star again
    #call new A* star fucntion with update cost and iterative as params as well
    while agent != end:
        adaptiveNode = adaptiveA(emptyMaze,agent,end,iterativeNode,updateCost)
        if(counter2 == 20):
            return "path not found"
        curr = adaptiveNode
        while curr is not None:
            agentPath.append(curr.pos)
            curr = curr.parent
        
        while i < len(agentPath):
            
            agent = agentPath[i]
            #print("agent is at ", agent)
            #generating next moves for agent and checking if we encounter a 1
            for move in actions:
                neighbor = (agent[0] + move[0], agent[1] + move[1])
        
                #within range?
                if  neighbor[1] > (len(maze[len(maze)-1]) -1) or neighbor[0] > (len(maze) - 1) or neighbor[1] < 0 or neighbor[0] < 0:
                    continue
        
            #   now that we know that its within range time to update neighbors
                if maze[neighbor[0]][neighbor[1]] == 1:
                    emptyMaze[neighbor[0]][neighbor[1]] = 1
                    #print(emptyMaze)
            
            
            #now to check if we can keep following this path agentPath's next item shall be our next move so 
            #check i + 1
            if(i+1) != len(agentPath):
                nextItem = agentPath[i+1]
                #print("next item is ", nextItem)
                if emptyMaze[nextItem[0]][nextItem[1]] == 1:
                    #print("empty maze value: ", emptyMaze[nextItem[0]][nextItem[1]])
                    #stop the loop
                    newStart = agent
                    i = len(agentPath)
                    #since the agent path isn't blocked we can keep going
            movedBlocks.append(agent);
            agentPath.remove(agent);
            
            numOFiters += 1
        
    
        while iterativeNode is not None:
            if newStart == currentNode.pos:
                updateCost = currentNode.cost
                break
            else:
                iterativeNode = iterativeNode.parent
        #after first pass of a start we need to update heuristics somehow
        #maybe call in A star in first pass
        # after that if not found create a new function which has our expanded list as a param
        #use the expanded list to update those nodes before we go again
        
        
        #problem here 
        #movedBlocks.append(agent);
        #print("mover Block is ", movedBlocks[0])
        #iterate thru the path aStar returned
        
            #print("num of iterations: ",numOFiters)
        #print("exited i loop")
        counter2 += 1
    return movedBlocks
def adaptiveA(maze, start, end, iterativeNode, updateCost):
        # Create start and end node
    start = Node(None, start)
    start.cost = start.manhattan = start.f = 0
    end = Node(None, end)
    end.cost = end.manhattan = end.f = 0

    # Initialize both open and closed list
    visited = []
    expanded = []

    # Tried using priority queue did not work rip
    #caved in and just used heapq too little time ;-;
    heapq.heapify(visited) 
    heapq.heappush(visited, start)

    counter = 0
    stopCondition = (len(maze) * len(maze[0]))*2

    
    North = (1, 0)
    South = (0, 1)
    East = (-1, 0)
    West = (0, -1)
    
    actions = (South,North,East,West)
    # Loop until you find the path to the goal
    visitedSize = len(visited)
    while visitedSize > 0:
        counter += 1

        if counter > stopCondition:
          #kept getting infinite loops 
          return None       
        
        # Get the current node
        currentState = heapq.heappop(visited)
        expanded.append(currentState)

        # Found the goal
        if currentState == end:
            return currentState

        # Generate children
        adaptive = 1
        possibleMoves = generatePossibleMoveset(actions, maze , currentState);
        
        
        # put into own function to reduce clutter
        # Loop through children
        visited = determineValidity2(possibleMoves, expanded, visited, currentState, end, adaptive, updateCost,iterativeNode)
        visitedSize = len(visited)
    return None
def updateF(node,targetNode, iterativeNode, updateCost):
        #set hvalue and g value
    gVal = node.cost;    
    hVal = calculateManhattan(node,targetNode)
        
    #calc fValue
    temp = iterativeNode
    fVal = 0 
    while temp is not None:
        if temp == targetNode:
            fVal = gVal + hVal + updateCost
        else:
            fVal = gVal + hVal
    
    return fVal
def Astar(maze, start, end):

   

    # Initialize both open and closed list
    visited = []
    expanded = []
    
    # Create start and end node
    start = Node(None, start)
    start.cost = start.manhattan = start.f = 0
    end = Node(None, end)
    end.cost = end.manhattan = end.f = 0
    
    # Tried using priority queue did not work rip
    #caved in and just used heapq too little time ;-;
    heapq.heapify(visited) 
    heapq.heappush(visited, start)

    counter = 0
    stopCondition = (len(maze) * len(maze[0]))*2

    
    North = (1, 0)
    South = (0, 1)
    East = (-1, 0)
    West = (0, -1)
    
    actions = (South,North,East,West)
    # Loop until you find the path to the goal
    visitedSize = len(visited)
    while visitedSize > 0:
        counter += 1

        if counter > stopCondition:
          #kept getting infinite loops 
          return None       
        
        # Get the current node
        currentState = heapq.heappop(visited)
        expanded.append(currentState)

        # Found the goal
        if currentState == end:
            return currentState

        # Generate possible next state
        possibleMoves = generatePossibleMoveset(actions, maze , currentState);
        
        
        # put into own function to reduce clutter
        # Loop through next states to find valid options to add to visited list
        visited = determineValidity(possibleMoves, expanded, visited, currentState, end)
        visitedSize = len(visited)

    return None
def calculateManhattan(node,targetNode):
    #intialize cordinates
    xCord_cell = node.pos[0];
    yCord_cell = node.pos[1];
    xCord_target = targetNode.pos[0];
    yCord_target = targetNode.pos[1];
    
    
    #calc difference
    xVal = abs(xCord_cell - xCord_target)
    yVal = abs(yCord_cell - yCord_target)
        
    #calc absolute values plus the sum
    #xVal = abs(xVal)
    #yVal = abs(yVal)
    hValue = xVal + yVal
        
    #return val
    return hValue

#F(s) = G(s) + H(s)
def calculateF(node,targetNode):
    #set hvalue and g value
    gVal = node.cost;    
    hVal = calculateManhattan(node,targetNode)
        
    #calc fValue
    fVal = gVal + hVal
    return fVal
import timeit
def main(): 
    maze3 =   [[0,0,0,0,0],
              [0,0,1,0,0],
              [0,0,1,1,0],
              [0,0,1,1,0],
              [0,0,0,1,0]]
    
    start =  (0,0)
    end =    (4,4)
      
    pathStr = 'E:/Coding/python work/project1/arrs/randGrid/' + str(1) + '.txt'
    with open(pathStr, 'r') as f:
            forwardMaze = [[int(num) for num in line.split(' ') * 2] for line in f]
    
    
    
    print("Repeated forward...")
    path = repeatedForward(forwardMaze, (10,10), end)
    path.insert(0, (0,0))
    print(path)
    print("\n\n\n")
    
    print("Repeated backward...")
    path = repeatedBackward(maze3, start, end)
    print(path)
    print("\n\n\n")
    
    
    print("Repeated adaptive...")
    path = adaptive(maze3, start, end)
    print(path)
    
    i = 0;
    ForwardrunTimeList = []
    while i < 50:
        pathStr = 'E:/Coding/python work/project1/arrs/randGrid/' + str(i) + '.txt'
        
        print(pathStr)
        with open(pathStr, 'r') as f:
            forwardMaze = [[int(num) for num in line.split(' ') * 2] for line in f]
        
        startTime = timeit.default_timer()
        start = (5,7)
         #end = (len(maze)-1, len(maze[0])-1)

        #path = astar(maze, start, end)

        end = (10,10)
    
        path = repeatedForward(forwardMaze, start, end)
        print(path)
    
        #path = repeatedBackward(maze3, start, end)
        #  print("repeated Backward:")
        #print(path)
        stopTime = timeit.default_timer()   
        print('Time: ', stopTime - startTime)
        totalTime = stopTime - startTime
        
        #add time to runTime list
        ForwardrunTimeList.append(totalTime)
        i += 1
    
    
    
    
    BackwardrunTimeList = []
    #running repeated backward on the 50 mazes generated
    i = 0
    while i < 50:
        pathStr = 'E:/Coding/python work/project1/arrs/randGrid/' + str(i) + '.txt'
        
        print(pathStr)
        
        
        with open(pathStr, 'r') as f:
            backwardMaze = [[int(num) for num in line.split(' ') * 2] for line in f]
        
        startTime = timeit.default_timer()
        start = (5,7)
         #end = (len(maze)-1, len(maze[0])-1)

        #path = astar(maze, start, end)

        end = (10,10)
    
        path = repeatedBackward(backwardMaze, start, end)
        print(path)
    
        #path = repeatedBackward(maze3, start, end)
        #  print("repeated Backward:")
        #print(path)
        stopTime = timeit.default_timer()   
        print('Time: ', stopTime - startTime)
        totalTime = stopTime - startTime
        
        #add time to runTime list
        BackwardrunTimeList.append(totalTime)
        i += 1
    
    
    
    
    AdaptiverunTimeList = []
    #running repeated backward on the 50 mazes generated
    i = 0
    while i < 50:
        pathStr = 'E:/Coding/python work/project1/arrs/randGrid/' + str(i) + '.txt'
        
        print(pathStr)
        
        
        with open(pathStr, 'r') as f:
            Maze = [[int(num) for num in line.split(' ') * 2] for line in f]
        
        startTime = timeit.default_timer()
        start = (0,0)
         #end = (len(maze)-1, len(maze[0])-1)

        #path = astar(maze, start, end)

        end = (4,4)
    
        path = adaptive(Maze, start, end)
        print(path)
    
        #path = repeatedBackward(maze3, start, end)
        #  print("repeated Backward:")
        #print(path)
        stopTime = timeit.default_timer()   
        print('Time: ', stopTime - startTime)
        totalTime = stopTime - startTime
        
        #add time to runTime list
        AdaptiverunTimeList.append(totalTime)
        i += 1
    
    ForwardrunTimeList = ForwardrunTimeList[::]
    print("Forward runtime list is ", ForwardrunTimeList)
    
    BackwardrunTimeList = BackwardrunTimeList[::]
    print("Backward runtime list is ", BackwardrunTimeList)
    
    AdaptiverunTimeList = AdaptiverunTimeList[::]
    print("Adaptive runtime list is ",  AdaptiverunTimeList)
    

if __name__ == "__main__":
    main();