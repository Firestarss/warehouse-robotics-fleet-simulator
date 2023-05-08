from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from evaluation import *
from visualization import *
from task_lists import *


from pytictoc import TicToc
import numpy as np
import matplotlib.pyplot as plt

evaluator = Evaluator()

# evaluator.set_static("warehouse_x",  4)
# evaluator.set_static("warehouse_y",  8)

# evaluator.set_variable("warehouse_x",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_y",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_z",  min=50, max=100, step=10)

evaluator.set_static("n_drone", 10)
evaluator.set_static("n_amr", 2)
# evaluator.set_variable("n_drone",  min=10, max=100, step=10)
# evaluator.set_variable("n_amr", min=1,  max=15,  step=5)

visualizer_info = []
master_dists = []
for i, wh_info in enumerate(evaluator.generate_wh_info()):
    # Initialize a warehouse and pick and drop points
    wh_map = WarehouseMap(wh_info, resolution=0.1, units="ft")
    wh_pick_points, wh1_drop_points = wh_map.generate_points()
    # wh1_map.show_occ_matrix(0)

    # Initialize task list for this warehouse
    task_list = TaskList()
    # task_list.populate_randomly(wh_pick_points, wh1_drop_points, 10)
    print(task_list)
    
    task_list = task_lists[i]
    
    print(f"Warehouse Dimensions: {wh_info['warehouse_x']}x {wh_info['warehouse_y']}y {wh_info['warehouse_z']}z")

    dists = []
    for k, fleet_comp in enumerate(evaluator.generate_fleet_comp()):
        print(f"    Fleet Composition: Drones = {fleet_comp[0][1]}, AMRs = {fleet_comp[1][1]}")
        
        fleet = Fleet()
        start_locations = [Point(p.x,p.y,5) for p in wh_pick_points]
        fleet.populate_by_composition(fleet_comp, start_locations)

        # task_allocator = TaskAllocator(task_list, fleet, wh_map.resolution,
        #                             region_type="homogeneous",
        #                             handoff_type="no_handoff")
        
        task_allocator = TaskAllocator(task_list, fleet, wh_map.resolution,
                                    region_type="sized_regions",
                                    handoff_type="closest2drop_handoff")

        task_allocator.cluster_regions()

        path_planner = PathPlanner(wh_map, fleet)

        for i, r in enumerate(task_allocator.regions):
            print("="*30, f"Planning region {i}/{len(task_allocator.regions)}")
            # print(f"Allocating task for region {r}...")
            task_allocator.allocate_tasks(r)
            # print(f"Planning paths for region {r}...")
            # path_planner.plan_next_region()

        # Save parameters relevant to visualization info so any simulation can be chosen for viewing
        visualizer_info.append([wh_map, task_list, fleet])

    #     dist = task_allocator.get_dist()
    #     dists.append([fleet_comp[0][1], fleet_comp[1][1], dist])
    # master_dists.append(dists)

# print(np.array(master_dists))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# for dist_list in master_dists:
#     x = [point[0] for point in dist_list]
#     y = [point[1] for point in dist_list]
#     z = [point[2] for point in dist_list]

#     ax.scatter(x, y, z, alpha=1)

# plt.show()

wh_map, task_list, fleet = visualizer_info[0]

# task_visualizer = Visualizer(wh_map, task_list, fleet, vis_type="static_with_robots", show_task_labels=False)
# task_visualizer.show()

# print(fleet.get_robots_as_list("Drone"))

# print(fleet)

visualizer = Visualizer(wh_map, task_list, fleet, vis_type="static_no_robots", task_plot_mode="detailed", split_tasks=True, show_task_labels=False)
visualizer.show()