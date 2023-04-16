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
            "x" : 8,
            "y" : 5
    },
    "aisle" : {
            "x" : 50,
            "y" : 20
    },
    "border" : {
            "top" : 20,
            "down" : 20,
            "left" : 10,
            "right" : 10
    },
    "height" : 60,
    "pick_points" : {
            "shelf_buffer": 5,
            "neighbor_buffer": 5
    },
    "drop_points" : {
            "top" : True,
            "down" : False,
            "left" : False,
            "right" : False
    }
}

wh1_map = WarehouseMap(wh1_info, resolution=0.1, units="ft")
wh1_pick_points, wh1_drop_points = wh1_map.generate_points()
# wh1_map.show_occ_matrix(0)

rand_task_list = TaskList()
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 50)
# print(rand_task_list)

fleet = Fleet()
fleet.populate_by_composition([["Drone", 20], ["AMR", 2]], wh1_pick_points)
# print(fleet)

task_allocator = TaskAllocator(rand_task_list, fleet)
task_allocator.populate_fleet(allocation_type="regional")


# fleet.robots["Drone"]["D0"].add_task(rand_task_list.tasks[0])

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


task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="color_tasks_traces_off")
task_visualizer.show()


path_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="black_tasks_traces_on", show_t=True)
path_visualizer.show()
