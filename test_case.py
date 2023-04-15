from utils import *
from warehouse_map import *
from robot_fleet import *

class TestCase:
    def __init__(self, fleet, task_list, wh_map, task_allocator, path_planner, 
                 visualizer, evaluator, delta_t, max_sim_time):
        self.fleet = fleet
        self.task_list = task_list
        self.wh_map = wh_map
        self.task_allocator = task_allocator
        self.path_planner = path_planner
        self.visualizer = visualizer
        self.evaluator = evaluator
        self.delta_t = delta_t
        self.max_sim_time = max_sim_time
        self.evaluation = None

    def __repr__(self):
        return (f"TestCase(\nfleet={self.fleet},\n" +
                f"task_list={self.task_list},\n" +
                f"wh_map={self.wh_map},\n" +
                f"task_allocator={self.task_allocator},\n" +
                f"path_planner={self.path_planner},\n" +
                f"visualizer={self.visualizer},\n" +
                f"evaluator={self.evaluator},\n" +
                f"delta_t={self.delta_t}, max_sim_time={self.max_sim_time})")
