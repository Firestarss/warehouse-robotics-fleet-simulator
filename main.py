from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from visualization import *

wh1_info = {
    "shelf_size": {
            "x" : 20,
            "y" : 50,
            "z" : 40
            },
    "shelf_count" : {
            "x" : 4,
            "y" : 2
    },
    "aisle" : {
            "x" : 20,
            "y" : 20
    },
    "border" : {
            "top" : 20,
            "down" : 20,
            "left" : 10,
            "right" : 10
    }
}

wh1_map = WarehouseMap(wh1_info, resolution=0.1, units="ft")
wh1_pick_points, wh1_drop_points = wh1_map.generate_points()

# wh1_map.show_occ_matrix(0)

rand_task_list = TaskList()
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 30)
# print(rand_task_list)

fleet = Fleet()
fleet.populate_by_composition([["Drone", 20], ["AMR", 2]], wh1_pick_points)
# print(fleet)

# print(fleet.get_robots_as_list("Drone"))

# fleet.robots["Drone"]["D0"].add_paths([[Point(5,25,5),
#                                         Point(5,35,5),
#                                         Point(5,45,5),
#                                         Point(5,55,5)]])
# fleet.robots["Drone"]["D0"].add_paths([[Point(5,55,5),
#                                         Point(5,55,15),
#                                         Point(5,55,25)]])
# fleet.robots["Drone"]["D0"].add_paths([[Point(5,55,25),
#                                         Point(5,65,25),
#                                         Point(5,75,25),
#                                         Point(5,85,25)]])
# print(fleet.robots["Drone"]["D0"].path)
# print(fleet.closest_robots_at_end_path(Point(20,15,5), "Drone"))


# 1 <= n_ground_agents <= n_drone_agents <= n_tasks

task_allocator = TaskAllocator(rand_task_list, fleet)

# Put arguments in this funciton
task_allocator.populate_fleet(allocation_type="regional")

# fleet.robots["Drone"]["D0"].add_task(rand_task_list.tasks[0])


task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="color_tasks_traces_off")
task_visualizer.show()


path_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="black_tasks_traces_on", show_t=True)
path_visualizer.show()
