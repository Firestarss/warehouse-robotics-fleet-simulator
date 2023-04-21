from utils import *
from warehouse_map import *
from robot_fleet import *
import numpy as np
import heapq
import itertools

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, position=None, parent=None):
        self.position = position
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0

    def get_position(self): return self.position

    def set_position(self, value): self.position = value

    def get_parent(self): return self.parent

    def set_parent(self, value): self.parent = value

    def get_g(self): return self.g

    def set_g(self, value):
        self.g = value
        self.update_f()

    def get_h(self): return self.h

    def set_h(self, value):
        self.h = value
        self.update_f()

    def get_f(self): return self.f

    def update_f(self): self.f = self.g + self.h

    def __eq__(self, other) -> bool:
        return self.position == other.position
    
    def __ne__(self, other) -> bool:
        return not self.__eq__(other)
    
    def __hash__(self) -> int:
        return hash(self.position)
    
    def __repr__(self) -> str:
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

class PathPlanner:
    def __init__(self, map, fleet):
        self.map = map
        self.fleet = fleet

        self.colors = {
            "HEADER" : '\033[95m',
            "OKBLUE" : '\033[94m',
            "OKCYAN" : '\033[96m',
            "OKGREEN" : '\033[92m',
            "WARNING" : '\033[93m',
            "FAIL" : '\033[91m',
            "ENDC" : '\033[0m',
            "BOLD" : '\033[1m',
            "UNDERLINE" : '\033[4m'
        }
        
    def __repr__(self):
        pass

    def backtrack_path(self, node):
        path = []
        cur_node = node
        while cur_node is not None:
            path.append(cur_node.position)
            cur_node = cur_node.parent
        
        return [self.map.cell_to_point_center(Cell(x, y, z)) for x,y,z in path[::-1]]

    def calc_a_star_path(self, start: Point, end: Point, collisions = True, debug_info = None):
        start_cell = self.map.point_to_cell(start)
        end_cell = self.map.point_to_cell(end)

        start_t = tuple(start_cell)
        end_t = tuple(end_cell)
        
        start_node = Node(tuple(self.map.point_to_cell(start)))
        end_node = Node(tuple(self.map.point_to_cell(end)))

        open_list = [start_node]
        closed_set = set()

        heapq.heapify(open_list)

        adjacent_deltas = set(itertools.product((-1,0,1), (-1,0,1), (-1,0,1)))
        adjacent_deltas.discard((0,0,0))

        x_lim = self.map.wh_zone.x_lims[1] * self.map.resolution
        y_lim = self.map.wh_zone.y_lims[1] * self.map.resolution
        z_lim = self.map.wh_zone.z_lims[1] * self.map.resolution
        

        while len(open_list) > 0:
            # Get the current node
            cur_node = heapq.heappop(open_list)
            closed_set.add(cur_node)

            # If the goal is found
            if cur_node == end_node:
                return self.backtrack_path(cur_node)
            
            # Generate children
            children = []

            for delta in adjacent_deltas:
                neighbor = tuple(cur_node.get_position()[i] + delta[i] for i in range(len(cur_node.get_position())))

                # Check if in range
                lt_zero = [neighbor[i] < 0 for i in range(3)]
                gt_lim = [neighbor[0] >= x_lim, neighbor[1] >= y_lim, neighbor[2] >= z_lim]
                if True in lt_zero or True in gt_lim:
                    continue

                # Check if cell is blocked (if collisions flag is True)
                if collisions:
                    cur_cell = Cell(neighbor[0], neighbor[1], neighbor[2])
                    if self.map.cell_blocked(cur_cell):
                        continue

                # Create and add new node to children
                new_node = Node(neighbor, cur_node)
                children.append(new_node)


            for child in children:
                if child in closed_set:
                    continue

                if child in open_list:
                    continue

                c_x, c_y, c_z = child.get_position()
                child_cell = Cell(c_x, c_y, c_z)
                

                child.set_g(cur_node.get_g() + 1)
                child.set_h(diag_dist(child_cell, end_cell))

                heapq.heappush(open_list, child)

        print(f"{self.colors['FAIL']}No path found: {start} --> {end} || {start_cell} --> {end_cell} || {debug_info}{self.colors['ENDC']}")
        return None
    
    def calc_a_star_path_without_collisions(self, start: Point, end: Point, debug_info = None):
        return self.calc_a_star_path(start, end, False, debug_info = debug_info)


    def calc_manhattan_path(self, start: Point, end: Point, debug_info = None):
        s_x, s_y, s_z = self.map.point_to_cell(start)
        e_x, e_y, e_z = self.map.point_to_cell(end)

        path = []

        for x in np.arange(
                        start = s_x,
                        stop = e_x,
                        step = -1 if s_x > e_x else 1,
                        dtype = int):
            path.append(Cell(x, s_y, s_z))

        s_x = e_x
        
        for y in np.arange(
                        start = s_y,
                        stop = e_y,
                        step = -1 if s_y > e_y else 1,
                        dtype = int):
            path.append(Cell(s_x, y, s_z))

        s_y = e_y

        for z in np.arange(
                        start = s_z,
                        stop = e_z ,
                        step = -1 if s_z > e_z else 1,
                        dtype = int):
            path.append(Cell(s_x, s_y, z))

        path.append(self.map.point_to_cell(end))

        return [self.map.cell_to_point_center(node) for node in path]

    def temp_plan_all_paths(self, alg = "manhattan"):
        algorithms = {
            "manhattan": self.calc_manhattan_path,
            "a*": self.calc_a_star_path,
            "a*_no_col": self.calc_a_star_path_without_collisions
        }
        path_planning_alg = algorithms[alg]

        robots_with_tasks = self.fleet.get_robots_with_tasks()
        for robot in robots_with_tasks:
            robot.get_next_task().started = True
            for task in robot.task_list.tasks:
                debug_info = [task.task_id, task.assigned_robot.robot_id]
                path_segment = path_planning_alg(task.pick_point, task.drop_point, debug_info = debug_info)
                robot.add_path_segment(path_segment)
                task.picked = True
                task.done = True
                next_task = robot.get_next_task()
                if next_task:
                    path_segment = path_planning_alg(task.drop_point, next_task.pick_point, debug_info = debug_info)
                    robot.add_path_segment(path_segment)
                    next_task.started = True

    
    # def plan_all_paths(self):
    #     while not self.fleet.is_fleet_done():
    #         robots_with_tasks = self.fleet.get_robots_with_tasks()

    #         for robot in robots_with_tasks:
    #             if robot.get_current_task() is None:
    #                 robot.get_next_task().started = True