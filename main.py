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
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 10)
# print(rand_task_list)

fleet = Fleet()
fleet.populate_by_composition([["Drone", 5], ["AMR", 2]], wh1_pick_points)
print(fleet)

task_allocator = TaskAllocator(rand_task_list, fleet)
task_allocator.populate_fleet()

path_planner = PathPlanner(wh1_map, fleet)
path_planner.temp_plan_all_paths()

print([robot.path for robot in fleet.get_robots_as_list()])
# task_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="color_tasks_traces_off")
# task_visualizer.show()


path_visualizer = Visualizer(wh1_map, rand_task_list, fleet, vis_type="black_tasks_traces_on", show_t=False)
path_visualizer.show()
