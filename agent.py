from collections import deque
import heapq
import math
from operator import pos
from tracemalloc import start 

class SearchAgent:

    def __init__(self):
        self.plan = []
        self.active_algo = 'BFS'

    def manhattan_distance(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def euclidean_distance(self, pos, goal):
        return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)       

    def get_neighbors(self, position, grid_size, walls):
        x, y = position
        width, height = grid_size

        possible_moves = [
            ('Up', (x, y + 1)),
            ('Down', (x, y - 1)),
            ('Left', (x - 1, y)),
            ('Right', (x + 1, y))
        ]

        neighbors = []

        for action, new_position in possible_moves:
            nx, ny = new_position

            if (
                0 <= nx < width
                and 0 <= ny < height
                and new_position not in walls
            ):
                neighbors.append((new_position, action))

        return neighbors

    def bfs_search(self, start, goal, grid_size, walls):

        frontier = deque()
        frontier.append((start, []))

        reached = {start}

        while frontier:

            current, path = frontier.popleft()

            if current == goal:
                return path

            for next_position, action in self.get_neighbors(
                current, grid_size, walls
            ):

                if next_position not in reached:
                    reached.add(next_position)

                    new_path = path + [action]

                    frontier.append(
                        (next_position, new_path)
                    )

        return []

    def dfs_search(self, start, goal, grid_size, walls):

        frontier = []
        frontier.append((start, []))

        reached = {start}

        while frontier:

            current, path = frontier.pop()

            if current == goal:
                return path

            for next_position, action in self.get_neighbors(
                current, grid_size, walls
            ):

                if next_position not in reached:
                    reached.add(next_position)

                    new_path = path + [action]

                    frontier.append(
                        (next_position, new_path)
                    )

        return []

    def ucs_search(self, start, goal, grid_size, walls):

        frontier = []

        heapq.heappush(
            frontier,
            (0, start, [])
        )

        reached = {start: 0}

        while frontier:

            cost, current, path = heapq.heappop(frontier)

            if current == goal:
                return path

            for next_position, action in self.get_neighbors(
                current, grid_size, walls
            ):

                new_cost = cost + 1

                if (
                    next_position not in reached
                    or new_cost < reached[next_position]
                ):

                    reached[next_position] = new_cost

                    new_path = path + [action]

                    heapq.heappush(
                        frontier,
                        (new_cost, next_position, new_path)
                    )

        return []

    def astar_search(self, start, goal, grid_size, walls, heuristic_type='manhattan'):

        frontier = []
        reached = {}

        # Choose heuristic
        if heuristic_type == 'manhattan':
            h = self.manhattan_distance(start, goal)
        else:
            h = self.euclidean_distance(start, goal)

        # (f, g, position, path)
        heapq.heappush(frontier, (h, 0, start, []))
        reached[start] = 0

        while frontier:

            f_cost, g_cost, current, path = heapq.heappop(frontier)

            if current == goal:
                return path

            for next_position, action in self.get_neighbors(current, grid_size, walls):

                new_g = g_cost + 1

                if next_position not in reached or new_g < reached[next_position]:

                    reached[next_position] = new_g

                    # calculate heuristic
                    if heuristic_type == 'manhattan':
                        h = self.manhattan_distance(next_position, goal)
                    else:
                        h = self.euclidean_distance(next_position, goal)

                    new_f = new_g + h
                    new_path = path + [action]

                    heapq.heappush(frontier, (new_f, new_g, next_position, new_path))

        return []

    def sense_and_act(self, percept):

        if not self.plan:

            current_position = tuple(percept['agent_pos'])

            all_food = percept['all_food']

            if not all_food:
                return 'Stay'

            grid_size = percept['grid_size']

            walls = set(percept['walls'])

            target_food = min(
                all_food,
                key=lambda food:
                abs(food[0] - current_position[0])
                + abs(food[1] - current_position[1])
            )

            target_food = tuple(target_food)

            if self.active_algo == 'BFS':

                self.plan = self.bfs_search(
                    current_position,
                    target_food,
                    grid_size,
                    walls
                )

            elif self.active_algo == 'DFS':

                self.plan = self.dfs_search(
                    current_position,
                    target_food,
                    grid_size,
                    walls
                )

            elif self.active_algo == 'UCS':

                self.plan = self.ucs_search(
                    current_position,
                    target_food,
                    grid_size,
                    walls
                )

            elif self.active_algo == 'AStar':

                self.plan = self.astar_search(
                    current_position,
                    target_food,
                    grid_size,
                    walls,
                    heuristic_type='manhattan'   # you can switch later
            )

        if self.plan:
            return self.plan.pop(0)

        return 'Stay'