# Snake Game Solver

## Project Overview

This project implements various algorithms to solve the classic Snake game automatically. Three different search algorithms are used:
- A* Search Algorithm
- Greedy Best First Search Algorithm
- Depth First Search (DFS) Algorithm

Each algorithm is implemented as an agent that plans and executes movements to reach the food in the maze, avoiding obstacles and its own tail.

## Agents Implemented

### A* Search Agent 

This agent uses the A* search algorithm to find the shortest path to the food in the maze. It calculates the heuristic based on the Manhattan distance between the current position and the food.

### Greedy Best First Search Agent 

The Greedy Best First Search agent prioritizes nodes based on the heuristic of the next position to the food. It explores the most promising path first without considering the cost to reach the current position.

### Depth First Search Agent 

The Depth First Search agent explores as far as possible along each branch before backtracking. It may not always find the shortest path but explores deeper paths extensively.


