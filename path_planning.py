from utils import *
from warehouse_map import *
from robot_fleet import *
import numpy as np

class PathPlanner:
    def __init__(self, map, fleet):
        self.map = map
        self.fleet = fleet
        
    def __repr__(self):
        pass

    def calc_manhattan_path(self, start: Point, end: Point):
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
    
    def temp_plan_all_paths(self):
        robots_with_tasks = self.fleet.get_robots_with_tasks()
        for robot in robots_with_tasks:
            robot.get_next_task().started = True
            for task in robot.task_list.tasks:
                path_segment = self.calc_manhattan_path(task.pick_point, task.drop_point)
                robot.add_path_segment(path_segment)
                task.picked = True
                task.done = True
                next_task = robot.get_next_task()
                if next_task:
                    path_segment = self.calc_manhattan_path(task.drop_point, next_task.pick_point)
                    robot.add_path_segment(path_segment)
                    next_task.started = True

    
    # def plan_all_paths(self):
    #     while not self.fleet.is_fleet_done():
    #         robots_with_tasks = self.fleet.get_robots_with_tasks()

    #         for robot in robots_with_tasks:
    #             if robot.get_current_task() is None:
    #                 robot.get_next_task().started = True