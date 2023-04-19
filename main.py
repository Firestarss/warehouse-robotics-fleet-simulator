from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from visualization import *

wh1_info = {
    "bin" : {
        "x" : 10,
        "y" : 10,
        "z" : 10,
        "pick_distance" : 5
    },
    "shelf" : {
        "x" : 2,
        "y" : 5,
        "z" : 4
    },
    "warehouse" : {
        "x" : 5,
        "y" : 4,
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
