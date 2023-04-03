from utils import *
import numpy as np

class WarehouseMap:
    def __init__(self, wh_zone, blocked_areas, pick_points, drop_points, 
                 resolution, units="ft"):
        self.wh_zone = wh_zone
        self.blocked_areas = blocked_areas
        self.pick_points = pick_points
        self.drop_points = drop_points
        self.resolution = resolution # "pixels" per unit
        self.units = units
        self.occupancy_matrix = self.make_occupancy_matrix(self.resolution)

    def point_to_pixel(self, point):
        """
        Converts Point from x-y-z in feet to the row-col-layer of the closest pixel in the occupancy matrix.

        Args:
            point (Point): point to be converted

        Returns:
            Point: pixel where pixel.x is row, pixel.y is col, pixel.z is layer
        """
        pix_x = round((point.x - self.wh_zone.x_lims[0]) * self.resolution)
        pix_y = round((point.y - self.wh_zone.y_lims[0]) * self.resolution)
        pix_z = round((point.z - self.wh_zone.z_lims[0]) * self.resolution)
        return Point(pix_x, pix_y, pix_z)

    def pixel_to_point(self, pixel):
        """
        Converts Point from row-col-layer to x-y-z in feet.

        Args:
            pixel (Point): pixel to be converted

        Returns:
            Point: point where x, y, and z are in feet
        """
        point_x = pixel.x/self.resolution + self.wh_zone.x_lims[0]
        point_y = pixel.y/self.resolution + self.wh_zone.y_lims[0]
        point_z = pixel.z/self.resolution + self.wh_zone.z_lims[0]
        return Point(point_x, point_y, point_z)

    def zone_points_to_pixels(self, point_zone):
        pix_c1 = self.point_to_pixel(point_zone.corners[0])
        pix_c2 = self.point_to_pixel(point_zone.corners[1])
        return Zone([pix_c1.x, pix_c2.x], 
                    [pix_c1.y, pix_c2.y], 
                    [pix_c1.z, pix_c2.z])

    def zone_pixels_to_points(self, pix_zone):
        point_c1 = self.pixel_to_point(pix_zone.corners[0])
        point_c1 = self.pixel_to_point(pix_zone.corners[1])
        return Zone([point_c1.x, point_c2.x], 
                    [point_c1.y, point_c2.y], 
                    [point_c1.z, point_c2.z])

    def make_occupancy_matrix(self, resolution):
        # TODO incoperate range not being an exact multiple of resolution
        matrix_x_range = int(self.wh_zone.x_range() * resolution)
        matrix_y_range = int(self.wh_zone.y_range() * resolution)
        matrix_z_range = int(self.wh_zone.z_range() * resolution)
        occ_matrix = np.full((matrix_x_range, matrix_y_range, matrix_z_range), False, dtype=bool)
        for zone in self.blocked_areas:
            zone_pix = self.zone_points_to_pixels(zone)
            occ_matrix[zone_pix.x_lims[0]:zone_pix.x_lims[1], 
                       zone_pix.y_lims[0]:zone_pix.y_lims[1], 
                       zone_pix.z_lims[0]:zone_pix.z_lims[1]] = True
        return occ_matrix

    def get_occ_matrix_layer(self, layer_num):
        matrix_layer = self.occupancy_matrix[:,:,layer_num].astype(int)
        return(matrix_layer)
    
    def show_occ_matrix(self, layer_num):
        space = "   "
        free = "•"
        blocked = "X"
        matrix_layer = self.occupancy_matrix[:,:,layer_num]
        num_rows = len(self.occupancy_matrix)
        num_cols = len(self.occupancy_matrix[0])

        line0 = "   "
        for x in range(num_rows):
            line0 += str(x) + space[:-1]
            if x < 10:
                line0 += " "
        lines = [line0]
        for j in range(num_cols):
            line = str(j) + " "
            if j < 10:
                line += " "
            for i in range(num_rows):
                if self.occupancy_matrix[i,j,layer_num]:
                    line += blocked + space
                else:
                    line += free + space
            lines.append(line)
        
        print(lines[0])
        for l in lines[1:]:
            print(l)
            
    def pixel_blocked(self, pixel):
        """
        Returns True if pixel is in an obstacle and False if not.

        Args:
            pixel (Point): pixel to check
        """
        if self.occupancy_matrix[pixel.x, pixel.y, pixel.z]:
            return True
        return False
    
    def point_blocked(self, point):
        """
        Returns True if point is in an obstacle and False if not.

        Args:
            point (Point): point to check
        """
        pixel = self.point_to_pixel(point)
        return self.pixel_blocked(pixel)
        
    
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
pick_points = []
for k in pick_zs:
    for j in pick_ys:
        for i in pick_xs:
            pick_points.append(Point(i,j,k))

drop_xs = [5, 25, 45, 65, 85, 105, 125, 145, 165, 185]
drop_points = []
for i in drop_xs:
    drop_points.append(Point(i, 5, 5))

wh_map = WarehouseMap(wh1_zone,shelves,pick_points,drop_points,0.1, "ft")
wh_map.show_occ_matrix(0)