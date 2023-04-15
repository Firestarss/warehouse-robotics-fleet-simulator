from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *

wh1_zone = Zone([0,200], [0,150], [0, 50])
shelf_x_range = 20
shelf_y_range = 50
shelf_z_range = 40
shelf_x0 = 10
shelf_y0 = 20
shelf_spacing = 20

shelves = []
x = shelf_x0
y = shelf_y0
z = 0
while y < wh1_zone.y_lims[1]:
    while x < wh1_zone.x_lims[1]:
        shelves.append(Zone([x, x+shelf_x_range], 
                            [y, y+shelf_y_range], 
                            [z, shelf_z_range]))
        x += shelf_x_range + shelf_spacing
    y += shelf_y_range + shelf_spacing
    x = shelf_x0

pick_xs = [5, 35, 45, 75, 85, 115, 125, 155, 165, 195]
pick_ys = [25, 35, 45, 55, 65, 95, 105, 115, 125, 135]
pick_zs = [5, 15, 25, 35, 45]
wh1_pick_points = []
for k in pick_zs:
    for j in pick_ys:
        for i in pick_xs:
            wh1_pick_points.append(Point(i,j,k))

drop_xs = [5, 25, 45, 65, 85, 105, 125, 145, 165, 185]
wh1_drop_points = []
for i in drop_xs:
    wh1_drop_points.append(Point(i, 5, 5))

wh1_map = WarehouseMap(wh1_zone, shelves, wh1_pick_points, wh1_drop_points, 
                       0.1, "ft")
wh1_map.show_occ_matrix(0)

rand_task_list = TaskList()
rand_task_list.populate_randomly(wh1_pick_points, wh1_drop_points, 300)
print(rand_task_list)

fleet = Fleet()
fleet.populate_by_composition([["Drone", 200], ["AMR", 10]], wh1_pick_points)
print(fleet)

# print(fleet.get_robots_as_list("Drone"))

# fleet.robots["Drone"]["D0"].add_paths([[Point(10,10,10)]])
# print(fleet.robots["Drone"]["D0"].path)
# print(fleet.closest_robots_at_end_path(Point(20,15,5), "Drone"))

# 1 <= n_ground_agents <= n_drone_agents <= n_tasks

task_allocator = TaskAllocator(rand_task_list, fleet)

# Put arguments in this funciton
task_allocator.populate_fleet(allocation_type="regional")

# for agent in fleet.robots["AMR"].values():
#     print(agent.task_list)