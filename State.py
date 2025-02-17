"""
A comparison of search stratigies for playing the snake game. 

The basic skeleton of the puzzle is given that repeatedly generates food for the snake at randomly selected positions until the snake dies by hitting itself or the algorithm generates an empty list of moves and hence fails to find a path from the present position of snake head to the food.

This puzzle is designe by Mirza Mubasher Baig from NUCES:FAST Lahore, Pakistan

The puzzle provides a basic skeleton for providing the snake interface and connects it with a default search algorithm. 

Requirenments:
Implement each of the six search stratigies and provide multiple instance of the search algorithm for playing the game.

You are encourged to create a multi-thread application that uses multiple instance of the puzzle running cuncurrently using different search strategies for comparteison
 
"""

import random

class Const:
    UNIT_SIZE = 10

class Vector:
    def __init__(self, X, Y):
        self.X = X
        self.Y = Y
    def show(self):
        print("[" , self.X, ", ", self.Y, "]")
    
    def Update(self, X, Y):
        self.X = X
        self.Y = Y
        
    def Add(self, Vec):
        self.X = self.X + Vec.X
        self.Y = self.Y + Vec.Y

class Maze:
    def __init__(self, PuzzleFileName):
        self.MAP = []
        self.LoadMaze(PuzzleFileName)
    def LoadMaze(self, filename):
        with open(filename, 'r') as f:
            line = f.readline()
            [self.HEIGHT, self.WIDTH] = [int(digit) for digit in line.split()]
            self.MAP = [[int(digit) for digit in line.split()] for line in f]

class Snake:
    def __init__(self, Color, HeadPositionX = 10, HeadPositionY = 10, HeadDirectionX = 1, HeadDirectionY = 0):
        self.Size = 1
        self.Body = [] #in this implementation the body is empty
        self.Color = Color
        self.HeadPosition = Vector(HeadPositionX, HeadPositionY)
        self.HeadDirection = Vector(HeadDirectionX, HeadDirectionY)
        self.score = 0
        self.isAlive = True
        self.foodConsumed = False
   
    def moveSnake(self, State):
        
        if(self.isAlive == False):
            return

        self.HeadPosition.Add(self.HeadDirection)

        r = self.HeadPosition.Y
        c = self.HeadPosition.X
        #print("R: ", r, "C: ", c, "Height: ", State.FoodPosition.Y, "Width: ", State.FoodPosition.X)
        snakeCommingFromDown = False
        if (State.FoodPosition.Y < r):
            snakeCommingFromDown = True
        if(r >= State.maze.HEIGHT or r < 0) or (c >= State.maze.WIDTH or c < 0):
            self.isAlive = False
        elif(State.maze.MAP[r][c] == -1):
            self.isAlive = False
        elif(c == State.FoodPosition.X and ((snakeCommingFromDown and (r - 1) == State.FoodPosition.Y) or r == State.FoodPosition.Y)):
            if not State.foodConsumed:
                self.score = self.score + 10
                State.foodConsumed = True

    
class SnakeState():
    def __init__(self, Color, HeadPositionX, HeadPositionY, HeadDirectionX, HeadDirectionY, mazeFileName):
        self.snake = Snake(Color, HeadPositionX, HeadPositionY, HeadDirectionX, HeadDirectionY)
        self.maze = Maze(mazeFileName)
        self.generateFood()
    
    def generateFood(self):
        """
        Method to randomly place a circular 'food' object anywhere on Canvas.
        The tag on it is used for making decisions in the program
        """
        self.foodConsumed = False
        FoodFlag = False
        while (FoodFlag == False):
            x = random.randrange(3, self.maze.WIDTH  - 3)
            y = random.randrange(3, self.maze.HEIGHT - 3)

            if(self.maze.MAP[y][x] != -1):
                FoodFlag = True
                
        self.FoodPosition = Vector(x,y)
    
    def printState(self):
        # Printing the current state of the snake and the maze
        #print("Snake Head Position:", end=" ")
        self.snake.HeadPosition.show()
        #print("Food Position:", end=" ")
        self.FoodPosition.show()
        # Printing the maze
        #print("Maze:")
        #for row in self.maze.MAP:
         #   print(row)