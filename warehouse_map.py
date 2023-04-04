from utils import *
import numpy as np

class WarehouseMap:
    def __init__(self, wh_zone, blocked_areas, pick_points, drop_points, 
                 resolution, units="ft"):
        self.wh_zone = wh_zone
        self.blocked_areas = blocked_areas
        self.pick_points = pick_points
        self.drop_points = drop_points
        self.resolution = resolution # "cells" per unit
        self.units = units
        self.occupancy_matrix = self.make_occupancy_matrix(self.resolution)
        
    def __repr__(self):
        return (f"WarehouseMap(wh_zone={self.wh_zone}\n" + 
                f"    blocked_areas={self.blocked_areas},\n" +
                f"    pick_points={self.pick_points},\n" + 
                f"    drop_points={self.drop_points},\n" + 
                f"    resolution={self.resolution})")

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