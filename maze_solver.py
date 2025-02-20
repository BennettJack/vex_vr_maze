# region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *
from queue import PriorityQueue
from collections import deque
import heapq

# Brain should be defined by default
brain = Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)


# endregion VEXcode Generated Robot Configuration
# ------------------------------------------
#
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
#
# ------------------------------------------

# Add project code in "main"
def main():
    # Set robot to max speed
    drivetrain.set_turn_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(100, PERCENT)
    w, h = 8, 8
    global maze
    global startX, startY
    global endX, endY
    global route
    global currentCell
    global graph
    global cellsToVisit
    # Set start and end locations
    startX, startY = 7, 4
    endX, endY = 0, 3
    # Instantiate maze
    brain.clear()
    route = []
    maze = []

    # Map the maze out in a 2D array, and add a new cell in each index
    for i in range(w):
        column = []
        for j in range(h):
            column.append(Cell(i, j))
        maze.append(column)

    # Instantiate graph representation of the maze
    graph = Graph()

    # Setup for the starting cell
    currentCell = maze[startX][startY]
    currentCell.gScore = 0
    currentCell.fScore = currentCell.gScore + currentCell.hScore
    cellsToVisit = []
    cellsToVisit.append(currentCell)

    # Pen down to mark path
    # Moves 10mm forward so robot is in middle of cell
    pen.move(DOWN)
    drivetrain.drive_for(FORWARD, 10, MM)

    # Scans the first cell
    scan_cell()

    # A Star is used to find the exit first
    a_star(maze)

    # Once A Star has found the exit, discover all other cells in the maze
    fill_maze(maze, graph)

    # Return to start from current location
    pathToStart = breadth_first_search(graph, currentCell, maze[startX][startY])
    for cell in pathToStart:
        visitCell(cell)

        # Finds path to end and marks it with green
    pathToEnd = breadth_first_search(graph, currentCell, maze[endX][endY])
    pen.set_pen_color(GREEN)
    for cell in pathToEnd:
        cell.partOfFastestRoute = True
        visitCell(cell)
    print_maze()
    print_fastest_path(pathToEnd)

    # Orientates robot to leave the maze
    foundExit = False
    while foundExit == False:
        if (front_distance.get_distance(MM) > 2999):
            drivetrain.drive_for(FORWARD, 250, MM)
            foundExit = True
        else:
            drivetrain.turn_for(RIGHT, 90, DEGREES)


# A function to print the fastest path
def print_fastest_path(path):
    brain.new_line()
    brain.print("Fastest path step count: " + str(len(path)))
    for cell in path:
        brain.new_line()
        cell.print_coords()
        brain.print(" --> ")


# A function which takes a cell object and allows the robot to face and move to it
def visitCell(neighbour):
    global currentCell
    global cellsToVisit
    if (neighbour.xCoordinates < currentCell.xCoordinates):
        drivetrain.turn_to_heading(0, DEGREES)
        drivetrain.drive_for(FORWARD, 250, MM)


    elif (neighbour.yCoordinates > currentCell.yCoordinates):
        drivetrain.turn_to_heading(90, DEGREES)
        drivetrain.drive_for(FORWARD, 250, MM)

    elif (neighbour.xCoordinates > currentCell.xCoordinates):
        drivetrain.turn_to_heading(180, DEGREES)
        drivetrain.drive_for(FORWARD, 250, MM)


    elif (neighbour.yCoordinates < currentCell.yCoordinates):
        drivetrain.turn_to_heading(270, DEGREES)
        drivetrain.drive_for(FORWARD, 250, MM)

    currentCell = maze[neighbour.xCoordinates][neighbour.yCoordinates]

    # if this cell hasn't been scanned yet, scan it
    if (currentCell.scanned is False):
        scan_cell()


# A function which implements the A Star algorithm
# This version is a little more greedy as it prioritises the cell which is 'closer' to the end as we will be
# discovering the whole maze anyway
def a_star(maze):
    global currentCell
    global graph
    path = []

    priorityQueue = []
    previousCell = currentCell
    count = 0

    if count == 0:
        path.append(currentCell)
    while True:
        # For each of the current cell's neighbours, find how many steps away they are from the start and how close they are to the end
        for neighbour in graph.graph[currentCell]:
            count += 1
            temp_g_score = currentCell.gScore + 1
            temp_f_score = temp_g_score + neighbour.hScore

            # If the temporary score to the neighbour is lower than the neighbour's current score, add it to the priority queue
            # This is done so that A Star doesn't try to backtrack to cells it has already visited
            if temp_f_score < neighbour.fScore:
                neighbour.gScore = temp_g_score
                neighbour.fScore = temp_f_score
                # Normally the f score would be the highest priority, however the heuristics score is used as the highest prio here
                heapq.heappush(priorityQueue, (neighbour.hScore, neighbour.fScore, count, neighbour,))

        # Retrieve the highest priority tuple, and get the cell value
        cellToVisit = heapq.heappop(priorityQueue)[3]

        # Slight optimisation to prevent robot from going to a cell which is known to be a dead end based on number of walls that
        # surrounds it
        isDeadEnd = True

        while isDeadEnd:
            if cellToVisit.wallCount < 3:

                isDeadEnd = False
            else:
                brain.print("is dead end")
                cellToVisit = heapq.heappop(priorityQueue)[3]

        # If the cell to visit is a direct neighbour of the current cell, visit it
        if cellToVisit in graph.graph[currentCell]:
            visitCell(cellToVisit)
            path.append(cellToVisit)

        # If the cell to visit is not a direct neighbour, find the fastest path there and visit it
        else:
            pathToFollow = breadth_first_search(graph, currentCell, cellToVisit)
            for cell in pathToFollow:
                visitCell(cell)

        # If the exit has been found, end A Star
        if down_eye.detect(RED):
            break


# A function to visit all unvisited cells in the fastest way
def fill_maze(maze, graph):
    # Whilst there are still cells to visit, find the shortest path to each unvisited cell
    # Find the shortest path, and follow the fastest path to it.
    while len(cellsToVisit) > 0:
        paths = []
        for cell in cellsToVisit:
            path = breadth_first_search(graph, currentCell, cell)
            paths.append(path)

        closestPath = paths[0]
        for path in paths:
            if (len(path) < len(closestPath)):
                closestPath = path

        for cell in closestPath:
            visitCell(cell)


# A class representing each cell
class Cell:
    def __init__(self, w, h):
        self.xCoordinates, self.yCoordinates = w, h
        # Up, right, down, left
        self.walls = [False, False, False, False]
        self.sideScanned = [False, False, False, False]
        self.wallCount = 0
        self.discovered = False
        self.scanned = False

        # Scores for A Star algorithm
        # Best case distance from exit (heuristic)
        self.hScore = abs(self.xCoordinates - endX) + abs(self.yCoordinates - endY)
        # Steps from start
        self.gScore = float('inf')
        self.fScore = self.gScore + self.hScore

        self.partOfFastestRoute = False

        # As the maze is 8x8, slightly more efficient to assume that the sides are walls
        if (self.xCoordinates == 0) and (self.yCoordinates != 3):
            self.walls[0] = True
        if (self.xCoordinates == 7) and (self.yCoordinates != 4):
            self.walls[2] = True
        if (self.yCoordinates == 0):
            self.walls[3] = True
        if (self.yCoordinates == 7):
            self.walls[1] = True

    # A function to print the coordinats of the cell
    def print_coords(self):
        brain.print("(X: ")
        brain.print(self.xCoordinates)
        brain.print(" , Y: ")
        brain.print(self.yCoordinates)
        brain.print(")")

    # A function to set the amount of walls a cell has based on what walls have been discovered
    def calculate_walls(self):
        tempWallCount = 0
        for wall in self.walls:
            if wall is True:
                tempWallCount += 1

        self.wallCount = tempWallCount


# A function which scans all four sides of a cell to discover walls and neighbours
# If a side has already been scanned due to a neighbouring cell being scanned, the wall
# will be ignored here
def scan_cell():
    global currentCell
    global graph
    global cellsToVisit

    # Ensures program doesn't go out of bounds of the maze array
    if (currentCell.xCoordinates - 1) == -1:
        currentCell.walls[0] = True
        currentCell.sideScanned[0] = True

    elif currentCell.sideScanned[0] is False:
        # References the cell above
        cellAbove = maze[currentCell.xCoordinates - 1][currentCell.yCoordinates]
        drivetrain.turn_to_heading(0, DEGREES)
        # If there is not a wall infront of the robot, add the cell ahead as a neighbour, and
        # Mark the neighbouring cells oposit side as scanned so it wont be scanned in future
        if (front_distance.get_distance(MM) > 70):
            if cellAbove.scanned is False:
                graph.add_edge(currentCell, cellAbove)
                graph.add_edge(cellAbove, currentCell)
                cellAbove.discovered = True
                cellAbove.sideScanned[2] = True
                # Stores the cell above to visit later when we need to discover whole maze
                if cellAbove not in cellsToVisit:
                    cellsToVisit.append(cellAbove)
        # If a wall is found, set the top wall of the current cell to true, and the opposite wall of the cell above to true
        else:
            currentCell.walls[0] = True
            currentCell.sideScanned[0] = True
            cellAbove.walls[2] = True
            cellAbove.sideScanned[2] = True
            cellAbove.calculate_walls()

    if (currentCell.yCoordinates + 1) == 8:
        currentCell.walls[1] = True
        currentCell.sideScanned[1] = True
        pass
    elif currentCell.sideScanned[1] is False:
        cellToRight = maze[currentCell.xCoordinates][currentCell.yCoordinates + 1]
        drivetrain.turn_to_heading(90, DEGREES)
        if (front_distance.get_distance(MM) > 70):
            if cellToRight.scanned is False:
                graph.add_edge(currentCell, cellToRight)
                graph.add_edge(cellToRight, currentCell)
                cellToRight.discovered = True
                cellToRight.sideScanned[3] = True
                if cellToRight not in cellsToVisit:
                    cellsToVisit.append(cellToRight)


        else:
            currentCell.walls[1] = True
            currentCell.sideScanned[1] = True
            cellToRight.walls[3] = True
            cellToRight.sideScanned[3] = True
            cellToRight.calculate_walls()

    if (currentCell.xCoordinates + 1) == 8:
        currentCell.walls[2] = True
        currentCell.sideScanned[2] = True
        pass
    elif currentCell.sideScanned[2] is False:
        drivetrain.turn_to_heading(180, DEGREES)
        cellBelow = maze[currentCell.xCoordinates + 1][currentCell.yCoordinates]
        if (front_distance.get_distance(MM) > 70):
            if cellBelow.scanned is False:
                graph.add_edge(currentCell, cellBelow)
                graph.add_edge(cellBelow, currentCell)
                cellBelow.discovered = True
                cellBelow.sideScanned[0] = True
                if cellBelow not in cellsToVisit:
                    cellsToVisit.append(cellBelow)

        else:
            currentCell.walls[2] = True
            currentCell.sideScanned[2] = True
            cellBelow.walls[0] = True
            cellBelow.sideScanned[0] = True
            cellBelow.calculate_walls()

    if (currentCell.yCoordinates - 1) == -1:
        currentCell.walls[3] = True
        currentCell.sideScanned[3] = True
        pass
    elif currentCell.sideScanned[3] is False:
        drivetrain.turn_to_heading(270, DEGREES)
        cellToLeft = maze[currentCell.xCoordinates][currentCell.yCoordinates - 1]
        if (front_distance.get_distance(MM) > 70):
            if cellToLeft.scanned is False:
                graph.add_edge(currentCell, cellToLeft)
                graph.add_edge(cellToLeft, currentCell)
                cellToLeft.discovered = True
                cellToLeft.sideScanned[1] = True
                if cellToLeft not in cellsToVisit:
                    cellsToVisit.append(cellToLeft)

        else:

            currentCell.walls[3] = True
            currentCell.sideScanned[3] = True
            cellToLeft.walls[1] = True
            cellToLeft.sideScanned[1] = True
            cellToLeft.calculate_walls()

    # Once the cell has been scanned,
    currentCell.scanned = True
    cellsToVisit.remove(currentCell)

    print_maze()


# A class which stores the maze in graph form
class Graph:
    def __init__(self, graph: dict = {}):
        self.graph = graph

    # Adds edges connecting each of cell1's neighbours
    def add_edge(self, cell1, cell2):
        if cell1 not in self.graph:
            self.graph[cell1] = []

        self.graph[cell1].append(cell2)


# A function to execute a breadth first search between two cells
def breadth_first_search(graph, startCell, targetCell):
    visited = {}
    queue = deque()

    visited[startCell] = None
    queue.append(startCell)

    # Whilst the queue is not empty, take the first element of the queue and explore it's neighbours
    # Once the neighbour has been visited, store it and visit the next neighbour
    # Once the target cell has been reached, reverse the visited path
    while queue:
        cell = queue.popleft()

        if cell == targetCell:
            path = []

            while cell:
                path.append(cell)
                cell = visited[cell]

            return path[::-1]

        for neighbour in graph.graph[cell]:
            if neighbour not in visited:
                visited[neighbour] = cell
                queue.append(neighbour)


# Prints a visual representation of the maze in the console
def print_maze():
    global currentCell
    brain.clear()
    currentCellRow = 0
    currentCellColumn = 0
    for row in maze:
        print_divider(row, currentCellRow)
        currentCellRow += 1
        print_barrier(row, currentCellColumn)
        currentCellColumn += 1
        wait(0.1, MSEC)

    endRow = maze[0]
    print_divider(endRow, currentCellRow)


# Prints dividers for the top and bottom of each row
# This function only accounts for the top wall of a cell
def print_divider(row, currentCellRow):
    brain.new_line()
    firstCellInRow = True
    # The first and last row of cells will always be walls
    if ((currentCellRow == 0) or (currentCellRow == 8)):
        for cell in row:
            if (firstCellInRow):
                brain.print("+----+")
                firstCellInRow = False
            else:
                brain.print("----+")
    else:
        # If the top wall of a cell is a wall, print a wall
        # The first cell in the row has an extra + for visual consistency
        for cell in row:
            if (firstCellInRow):
                if (cell.walls[0] == False):
                    brain.print("+    +")
                else:
                    brain.print("+----+")
                firstCellInRow = False


            else:
                if (cell.walls[0] == False):
                    brain.print("    +")
                else:
                    brain.print("----+")


# A function which prints barriers for each column
# This function only accounts for the right wall of a cell
def print_barrier(row, currentCellColumn):
    global currentCell
    firstCellInRow = True
    rowsPrinted = 0
    # rowsPrinted has a limit of two as two barriers are used to make a square
    while (rowsPrinted < 2):
        brain.new_line()
        for cell in row:
            # The first cell will always have an extra | to the left
            if (firstCellInRow):
                # If the cell has a wall to the right, print a wall
                # If the wall is part of the fastest route, print an X
                if (cell.walls[1] == True):
                    if (rowsPrinted == 0):
                        brain.print("|    |")
                    if (rowsPrinted == 1):
                        if cell.partOfFastestRoute:
                            brain.print("|  x |")
                        else:
                            brain.print("|    |")
                else:
                    if (rowsPrinted == 0):
                        brain.print("|     ")
                    if (rowsPrinted == 1):
                        if cell.partOfFastestRoute:
                            brain.print("|  x  ")
                        else:
                            brain.print("|     ")
                firstCellInRow = False
            # For cells that are not the first in the row
            elif (cell.walls[1] == True):
                if (rowsPrinted == 0):
                    brain.print("    |")
                if (rowsPrinted == 1):
                    if cell.partOfFastestRoute:
                        brain.print("  x |")
                    else:
                        brain.print("    |")

            else:
                if (rowsPrinted == 0):
                    brain.print("     ")
                if (rowsPrinted == 1):
                    if cell.partOfFastestRoute:
                        brain.print("  x  ")
                    else:
                        brain.print("     ")

        rowsPrinted += 1
        firstCellInRow = True


# VR threads — Do not delete
vr_thread(main)