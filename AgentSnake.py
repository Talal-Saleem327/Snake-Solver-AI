
class Agent(object):
	def SearchSolution(self, state):
		return []
		
class AgentSnake(Agent):    
	def SearchSolution(self, state):
		FoodX = state.FoodPosition.X
		FoodY = state.FoodPosition.Y

		HeadX = state.snake.HeadPosition.X 
		HeadY = state.snake.HeadPosition.Y 
		
		DR = FoodY - HeadY
		DC = FoodX - HeadX
		
		plan = []
		
		F = -1
		if(DR == 0 and state.snake.HeadDirection.X*DC < 0):
			plan.append(0)
			F = 6
			
		if(state.snake.HeadDirection.Y*DR < 0):
			plan.append(3)
			if(DC == 0):
				F = 9
			else:
				DC = DC - 1
		Di = 6
		if(DR < 0):
			Di = 0
			DR = -DR
		for i in range(0,int(DR)):
			plan.append(Di)
		Di = 3
		if(DC < 0):
			Di = 9
			DC = -DC
		for i in range(0,int(DC)):
			plan.append(Di)
		if(F > 0):
			plan.append(F)
			F = -1
			
		return plan
	
	def showAgent():
		print("A Snake Solver By MB")
		
# You code of agent goes here
# You must create three agents one using A*, second using greedy best first search and third using an uninformed algo of your choice to make a plan

###################################'''A* search algo'''##########################################
import heapq

class A_star_algo:
    def __init__(self, state):
        self.state = state
        self.maze = state.maze.MAP
        self.start = (state.snake.HeadPosition.X, state.snake.HeadPosition.Y)
        self.goal = (state.FoodPosition.X, state.FoodPosition.Y)

    def huristic_distance(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def moves_validity(self, pos):
        x, y = pos
        return 0 <= x < len(self.maze) and 0 <= y < len(self.maze[0]) and self.maze[y][x] != -1

    def scearch_algorithm(self):
        arr1 = []
        heapq.heappush(arr1, (0 + self.huristic_distance(self.start, self.goal), 0, self.start))
        previous = {}
        cost = {self.start: 0}

        while arr1:
            _, _, curr_point = heapq.heappop(arr1)

            if curr_point == self.goal:
                return self.path_making(previous, curr_point)

            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:  
                neighbor = (curr_point[0] + dx, curr_point[1] + dy)
                if self.moves_validity(neighbor):
                    cost_of_point = cost[curr_point] + 1

                    if neighbor not in cost or cost_of_point < cost.get(neighbor, float('inf')):
                        previous[neighbor] = curr_point
                        cost[neighbor] = cost_of_point
                        huristic_cost = cost_of_point + self.huristic_distance(neighbor, self.goal)
                        heapq.heappush(arr1, (huristic_cost, cost_of_point, neighbor))
        return []

    def path_making(self, previous, curr_point):
        path = []
        while curr_point in previous:
            path.append(curr_point)
            curr_point = previous[curr_point]
        path.append(self.start)
        path.reverse()
        return path

    def path_planning(self, path):
        maze_directions = {(1, 0): 3, (-1, 0): 9, (0, 1): 6, (0, -1): 0}
        plan = []
        for i in range(1, len(path)):
             curr, next = path[i - 1], path[i]
             direction = (next[0] - curr[0], next[1] - curr[1])
             if direction in maze_directions:
                  plan.append(maze_directions[direction])
        return plan

    def Maze_Solution(self):
        print("Maze path finding using A* search algorithm: ")
        path = self.scearch_algorithm()
        return self.path_planning(path)
class Agent(object):
    def SearchSolution(self, state):
        return []
class A_sterik_scearch_agent(Agent):
    def SearchSolution(self, state):
        print("Finding path using A* search algorithm: ")
        A_sterik_maze_solver = A_star_algo(state)
        plan = A_sterik_maze_solver.Maze_Solution()
        return plan
    
###############################'''greedy search algorithm''''''###################################
from queue import PriorityQueue
class Agent(object):
     def SearchSolution(self, state):
         return []
class AAgentSnake(Agent):
    def SearchSolution(self, state):
        return Greedy_Best_First_Search_Agent().SearchSolution(state)

   

class Greedy_Best_First_Search(Agent):
   def SearchSolution(self, state):
       print("Path finding using Greedy Best First Search Algorithm: ")
       greedy = Greedy_Best_First_Search_Agent()
       plan = greedy.SearchSolution(state)
       return plan
   def showAgent():
        print("A Snake Solver By MB")
class Greedy_Best_First_Search_Agent(Agent):
    def __init__(self):
        self.directions = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}

    def huristic_distance(self, state, next_pos):
        return abs(next_pos[0] - state.FoodPosition.X) + abs(next_pos[1] - state.FoodPosition.Y)

    def SearchSolution(self, state):
        start = (state.snake.HeadPosition.X, state.snake.HeadPosition.Y)
        food = (state.FoodPosition.X, state.FoodPosition.Y)
        priority_q = PriorityQueue()
        priority_q.put((self.huristic_distance(state, start), start))
        visited = set()
        parent = {}
        found = False

        while not priority_q.empty():
            _, curr_point = priority_q.get()
            if curr_point == food:
                found = True
                break

            visited.add(curr_point)
            for direction_name, (dx, dy) in self.directions.items():
                next_pos = (curr_point[0] + dx, curr_point[1] + dy)
                if (0 <= next_pos[0] < state.maze.WIDTH and 0 <= next_pos[1] < state.maze.HEIGHT and
                        state.maze.MAP[next_pos[1]][next_pos[0]] != -1 and next_pos not in visited):
                    priority = self.huristic_distance(state, next_pos)
                    priority_q.put((priority, next_pos))
                    parent[next_pos] = (direction_name, curr_point)

        if not found:
            print("No path found")
            return []

        path = []
        while curr_point != start:
            direction_name, prev = parent[curr_point]
            curr_point = prev
            if direction_name == 'up':
                path.append(0)
            elif direction_name == 'down':
                path.append(6)
            elif direction_name == 'right':
                path.append(3)
            elif direction_name == 'left':
                path.append(9)
        path.reverse()
        #print("Path: ", path)
        return path

#################################'''Depth First Solution (DFS)'''##################################
import copy

class DepthFirstSearchImplementation:
    def __init__(self, state):
        self.state = state
        self.maze = state.maze.MAP
        self.start = (state.snake.HeadPosition.X, state.snake.HeadPosition.Y)
        self.goal = (state.FoodPosition.X, state.FoodPosition.Y)

    def dfs(self):
        visited = set()
        stack = [(self.start, [])]  

        while stack:
            curr, path = stack.pop()

            if curr == self.goal:
                self.state.snake.score += 0
                #self.state.generateFood()
                return path

            visited.add(curr)

            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:  
                neighbor = (curr[0] + dx, curr[1] + dy)
                if 0 <= neighbor[0] < len(self.maze) and 0 <= neighbor[1] < len(self.maze[0]):
                    if self.maze[neighbor[0]][neighbor[1]] == -1:  
                        continue
                    if neighbor not in visited:
                        new_path = copy.copy(path)
                        new_path.append(neighbor)
                        stack.append((neighbor, new_path))

        return []  

    def convert_path_to_plan(self, path):
        plan = []
        for i in range(1, len(path)):
            current, next = path[i - 1], path[i]
            if next[0] == current[0] + 1:
                plan.append(3)  
            elif next[0] == current[0] - 1:
                plan.append(9)  
            elif next[1] == current[1] + 1:
                plan.append(6)  
            elif next[1] == current[1] - 1:
                plan.append(0)  
        return plan

    def SearchSolution(self):
        print("Searching for Solution using DFS")
        path = self.dfs()
        return self.convert_path_to_plan(path)

class Agent(object):
    def SearchSolution(self, state):
        return []
        

class DFS_Scearch_Agent(Agent):
    def SearchSolution(self, state):
        print("AgentSnake searching for solution...")
        dfs_solver = DepthFirstSearchImplementation(state)
        plan = dfs_solver.SearchSolution()
        return plan











  





 
	