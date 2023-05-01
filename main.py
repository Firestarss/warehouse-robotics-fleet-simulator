from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from visualization import *

from pytictoc import TicToc
import numpy as np
import matplotlib.pyplot as plt

t = TicToc()

wh1_info = {
    "bin" : {
        "x" : 10,
        "y" : 10,
        "z" : 10,
        "pick_distance" : 5
    },
    "shelf" : {
        "x" : 2,
        "y" : 10,
        "z" : 4
    },
    "warehouse" : {
        "x" : 4,
        "y" : 2,
        "z" : 60,
        "aisle_x" : 40,
        "aisle_y" : 30
    },
    "border_top" : {
        "padding" : 20,
        "drop_points" : True
    },
    "border_down" : {
        "padding" : 20,
        "drop_points" : False
    },
    "border_left" : {
        "padding" : 10,
        "drop_points" : False
    },
    "border_right" : {
        "padding" : 10,
        "drop_points" : False
    }
}

wh1_map = WarehouseMap(wh1_info, resolution=0.1, units="ft")
wh1_pick_points, wh1_drop_points = wh1_map.generate_points()
# wh1_map.show_occ_matrix(0)

rand_task_list = TaskList()
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 100)
# print(rand_task_list)


fleet = Fleet()
fleet.populate_by_composition([["Drone", 20], ["AMR", 5]], wh1_pick_points)
print(fleet)


task_allocator = TaskAllocator(rand_task_list, fleet, wh1_map.resolution,
                               region_type="sized_regions_hypercluster",
                               allocation_type="closest2drop_handoff")

task_allocator.cluster_regions()

path_planner = PathPlanner(wh1_map, fleet)

for r in task_allocator.regions:
    task_allocator.allocate_tasks(r)

    # path_planner.temp_plan_all_paths()

task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="fleet_tasks")
task_visualizer.show()


# path_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="color_tasks_traces_off", show_t=True)
# path_visualizer.show()