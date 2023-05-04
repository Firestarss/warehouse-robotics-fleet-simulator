from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from visualization2 import *
from evaluation import *

from pytictoc import TicToc
import numpy as np
import matplotlib.pyplot as plt

evaluator = Evaluator()

# evaluator.set_static("warehouse_x",  4)
# evaluator.set_static("warehouse_y",  8)
# evaluator.set_variable("warehouse_x",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_y",  min=2, max=10, step=2)

# evaluator.set_variable("warehouse_x",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_y",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_z",  min=50, max=100, step=10)

wh_info_list = evaluator.generate_wh_info()

# evaluator.set_static("n_drone", 20)
# evaluator.set_static("n_amr", 5)
evaluator.set_variable("n_drone",  min=10, max=100, step=10)
evaluator.set_variable("n_amr", min=1,  max=15,  step=5)

fleet_comp_list = evaluator.generate_fleet_composition()

visualizer_info = []
for wh_info in wh_info_list:
    # Initialize a warehouse and pick and drop points
    wh_map = WarehouseMap(wh_info, resolution=0.1, units="ft")
    wh_pick_points, wh1_drop_points = wh_map.generate_points()
    # wh1_map.show_occ_matrix(0)

    # Initialize task list for this warehouse
    rand_task_list = TaskList()
    rand_task_list.populate_randomly(wh_pick_points, wh1_drop_points, 100)
    
    print(f"Warehouse Dimensions: {wh_info['warehouse_x']}x {wh_info['warehouse_y']}y {wh_info['warehouse_z']}z")

    for fleet_comp in fleet_comp_list:
        print(f"    Fleet Composition: Drones = {fleet_comp[0][1]}, AMRs = {fleet_comp[1][1]}")
        
        fleet = Fleet()
        fleet.populate_by_composition(fleet_comp, wh_pick_points)

        task_allocator = TaskAllocator(rand_task_list, fleet, wh_map.resolution,
                                    region_type="sized_regions_hypercluster",
                                    allocation_type="closest2drop_handoff")

        task_allocator.cluster_regions()

        path_planner = PathPlanner(wh_map, fleet)

        for r in task_allocator.regions:
            task_allocator.allocate_tasks(r)

            # path_planner.temp_plan_all_paths()

        # Save parameters relevant to visualization info so any simulation can be chosen for viewing
        visualizer_info.append([wh_map, rand_task_list, fleet])

wh_map, rand_task_list, fleet = visualizer_info[0]

# task_visualizer = Visualizer(wh_map, rand_task_list, fleet, vis_type="fleet_tasks")
# task_visualizer.show()

# print(fleet.get_robots_as_list("Drone"))

# path_planner = PathPlanner(wh1_map, fleet)
# path_planner.temp_plan_all_paths()

# task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="fleet_tasks")
# task_visualizer.show()


# path_visualizer = Visualizer(wh_map, rand_task_list, fleet, vis_type="black_tasks_traces_on", show_t=True)
# path_visualizer.show()

# animation_visualizer = Visualizer(wh_map, rand_task_list, fleet, vis_type="color_tasks_traces_off", show_t=True)
# animation_visualizer.animation()