from utils import *
import numpy as np

class WarehouseMap:
    def __init__(self, wh_info, resolution=0.1, units="ft"):
        self.wh_info = wh_info
        self.resolution = resolution # "cells" per unit
        self.units = units

        self.pick_points = []
        self.drop_points = []

        self.wh_zone, self.blocked_areas = self.generate_map()

        self.occupancy_matrix = self.make_occupancy_matrix(self.resolution)
        
    def __repr__(self):
        return (f"WarehouseMap(wh_info={self.wh_info}\n" + 
                f"    resolution={self.resolution},\n" + 
                f"    units={self.units})")
    
    def generate_points(self):
        x_range = self.shelving_range["x"]
        y_range = self.shelving_range["y"]
        z_range = self.shelving_range["z"]

        shelf_size = self.wh_info["shelf_size"]
        aisle_size = self.wh_info["aisle"]

        buffer = self.wh_info["pick_points"]["shelf_buffer"]

        # Populate pick_xs: a list of possible pick point x values
        # performed by contatenation of 2 lists for the front and rear side of each shelf
        pick_xs_1 = np.arange(x_range[0]-buffer,                 x_range[1], shelf_size["x"]+aisle_size["x"])
        pick_xs_2 = np.arange(x_range[0]+buffer+shelf_size["x"], x_range[1], shelf_size["x"]+aisle_size["x"])
        pick_xs = list(np.append(pick_xs_1, pick_xs_2))

        # Populate pick_xs: a list of possible pick point y values
        # performed by loop through shelf rows to avoid populating isles
        pick_ys = []
        for row in range(self.wh_info["shelf_count"]["y"]):
            y_anchor = row * (shelf_size["y"]+aisle_size["y"]) + y_range[0]

            pick_ys_row = np.arange(y_anchor+buffer, y_anchor+shelf_size["y"], self.wh_info["pick_points"]["neighbor_buffer"])

            pick_ys.extend(list(pick_ys_row))

        pick_zs = np.arange(5,z_range[1],10)
        
        for k in pick_zs:
            for j in pick_ys:
                for i in pick_xs:
                    self.pick_points.append(Point(i,j,k))

        print(f"TOTAL PICK_POINTS = {len(self.pick_points)}")

        # Populate Drop locations between 1 or all warehouse sides
        if self.wh_info["drop_points"]["top"]:
            self.drop_points.extend([Point(i, 5, 5) for i in np.arange(5,self.wh_zone.x_lims[1],20)])
        
        if self.wh_info["drop_points"]["down"]:
            self.drop_points.extend([Point(self.wh_zone.x_lims[1]-5, i, 5) for i in np.arange(5,self.wh_zone.y_lims[1],20)])
        
        if self.wh_info["drop_points"]["left"]:
            self.drop_points.extend([Point(5, i, 5) for i in np.arange(5,self.wh_zone.y_lims[1],20)])

        if self.wh_info["drop_points"]["right"]:
            self.drop_points.extend([Point(i, self.wh_zone.y_lims[1]-5, 5) for i in np.arange(5,self.wh_zone.y_lims[1],20)])

        return self.pick_points, self.drop_points
    
    def generate_map(self):
        shelf_x = self.wh_info["shelf_size"]["x"]
        shelf_y = self.wh_info["shelf_size"]["y"]
        shelf_z = self.wh_info["shelf_size"]["z"]

        count_x = self.wh_info["shelf_count"]["x"]
        count_y = self.wh_info["shelf_count"]["y"]

        aisle_x = self.wh_info["aisle"]["x"]
        aisle_y = self.wh_info["aisle"]["y"]

        bdr_top = self.wh_info["border"]["top"]
        bdr_down = self.wh_info["border"]["down"]
        bdr_left = self.wh_info["border"]["left"]
        bdr_right = self.wh_info["border"]["right"]

        shelves = []
        z = 0
        for x in range(count_x):
            x = x * (shelf_x + aisle_x) + bdr_left

            for y in range(count_y):
                y = y * (shelf_y + aisle_y) + bdr_top

                shelves.append(Zone([x, x+shelf_x], 
                                    [y, y+shelf_y], 
                                    [z, shelf_z]))

        # Borders of shelving units
        xlim = (shelf_x * count_x) + (aisle_x * (count_x-1))
        ylim = (shelf_y * count_y) + (aisle_y * (count_y-1))
        zlim = self.wh_info["height"]

        wh_zone = Zone([0,xlim+bdr_left+bdr_right], [0,ylim+bdr_top+bdr_down], [0, zlim])
        
        # Global ranges of shelves used to assign pick points
        self.shelving_range = {
                    "x" : [bdr_left, xlim+bdr_left],
                    "y" : [bdr_top, xlim+bdr_top],
                    "z" : [z, shelf_z]
        }

        print(f"WAREHOUSE ZONE DIMENSIONS: {wh_zone}")

        return wh_zone, shelves

    def point_to_cell(self, point):
        """
        Converts Point from x-y-z in feet to the row-col-layer of the closest cell in the occupancy matrix.

        Args:
            point (Point): point to be converted

        Returns:
            Point: cell where cell.x is row, cell.y is col, cell.z is layer
        """
        cell_x = round((point.x - self.wh_zone.x_lims[0]) * self.resolution)
        cell_y = round((point.y - self.wh_zone.y_lims[0]) * self.resolution)
        cell_z = round((point.z - self.wh_zone.z_lims[0]) * self.resolution)
        return Cell(cell_x, cell_y, cell_z)

    def cell_to_point(self, cell):
        """
        Converts Point from row-col-layer to x-y-z in feet.

        Args:
            cell (Point): cell to be converted

        Returns:
            Point: point where x, y, and z are in feet
        """
        point_x = cell.x/self.resolution + self.wh_zone.x_lims[0]
        point_y = cell.y/self.resolution + self.wh_zone.y_lims[0]
        point_z = cell.z/self.resolution + self.wh_zone.z_lims[0]
        return Point(point_x, point_y, point_z)

    def zone_points_to_cells(self, point_zone):
        cell_c1 = self.point_to_cell(point_zone.corners[0])
        cell_c2 = self.point_to_cell(point_zone.corners[1])
        return CellZone([cell_c1.x, cell_c2.x], 
                        [cell_c1.y, cell_c2.y], 
                        [cell_c1.z, cell_c2.z])

    def zone_cells_to_points(self, cell_zone):
        point_c1 = self.cell_to_point(cell_zone.corners[0])
        point_c2 = self.cell_to_point(cell_zone.corners[1])
        return PointZone([point_c1.x, point_c2.x], 
                         [point_c1.y, point_c2.y], 
                         [point_c1.z, point_c2.z])

    def make_occupancy_matrix(self, resolution):
        # TODO incoperate range not being an exact multiple of resolution
        matrix_x_range = int(self.wh_zone.x_range() * resolution)
        matrix_y_range = int(self.wh_zone.y_range() * resolution)
        matrix_z_range = int(self.wh_zone.z_range() * resolution)
        occ_matrix = np.full((matrix_x_range, matrix_y_range, matrix_z_range), False, dtype=bool)
        for zone in self.blocked_areas:
            zone_cell = self.zone_points_to_cells(zone)
            occ_matrix[zone_cell.x_lims[0]:zone_cell.x_lims[1], 
                       zone_cell.y_lims[0]:zone_cell.y_lims[1], 
                       zone_cell.z_lims[0]:zone_cell.z_lims[1]] = True
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
            
    def cell_blocked(self, cell):
        """
        Returns True if cell is in an obstacle and False if not.

        Args:
            cell (Point): cell to check
        """
        if self.occupancy_matrix[cell.x, cell.y, cell.z]:
            return True
        return False
    
    def point_blocked(self, point):
        """
        Returns True if point is in an obstacle and False if not.

        Args:
            point (Point): point to check
        """
        cell = self.point_to_cell(point)
        return self.cell_blocked(cell)