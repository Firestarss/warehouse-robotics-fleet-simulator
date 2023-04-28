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

# DONE: No picks on first level
# DONE: Create a better way to get the last assigned position from a robot
# DONE: AMR pick point is under the closest drone target
# DONE: Merge AMR task_list items into a single task per region.
# TODO?: Cap region size by some global carrying capacity number

# drones = []
# amrs = []
# times = []

# for amr_count in range(1, 50):
#     for drone_count in range(1, 50):
#         if drone_count >= amr_count:
#             t.tic()
wh1_map = WarehouseMap(wh1_info, resolution=0.1, units="ft")
wh1_pick_points, wh1_drop_points = wh1_map.generate_points()
# wh1_map.show_occ_matrix(0)

rand_task_list = TaskList()
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 100)
# print(rand_task_list)


fleet = Fleet()
fleet.populate_by_composition([["Drone", 20], ["AMR", 5]], wh1_pick_points)
print(fleet)

# allocator = TaskAllocator(rand_task_list, fleet)
# allocator.populate_fleet(allocation_type="regional_base")

task_allocator = TaskAllocator(rand_task_list, fleet, allocation_type="homogeneous", resolution=wh1_map.resolution)

path_planner = PathPlanner(wh1_map, fleet)

for r in task_allocator.regions:
    
    task_allocator.allocate_tasks(r)

    # path_planner.temp_plan_all_paths()

            
# drones.append(drone_count)
# amrs.append(amr_count)
# times.append(t.tocvalue())

# x = np.asarray(drones)
# y = np.asarray(amrs)
# z = np.asarray(times)

# print(z)

# fig = plt.figure()

# ax = fig.add_subplot(111, projection='3d')

# ax.scatter(x,y,z)

# ax.set_xlabel('# Drones')
# ax.set_ylabel('# AMRS')
# ax.set_zlabel('t')

# ax.set_zlim(0, 0.04)

# plt.show()

# print(fleet.get_robots_as_list("Drone"))

# path_planner = PathPlanner(wh1_map, fleet)
# path_planner.temp_plan_all_paths()

task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="fleet_tasks")
task_visualizer.show()


# path_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="color_tasks_traces_off", show_t=True)
# path_visualizer.show()