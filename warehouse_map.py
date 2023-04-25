from utils import *
import numpy as np

class WarehouseMap:
    def __init__(self, wh_info, resolution=0.1, units="ft"):
        self.resolution = resolution # "cells" per unit
        self.units = units

        self.pick_points = []
        self.drop_points = []

        self.initialize_info(wh_info)

        self.blocked_areas, self.shelf_zone, self.wh_zone = self.generate_zones()

        self.occupancy_matrix = self.make_occupancy_matrix(self.resolution)
        
    def __repr__(self):
        return (f"WarehouseMap(wh_info=wh_info\n" + 
                f"    resolution={self.resolution},\n" + 
                f"    units={self.units})")
    
    def initialize_info(self, wh_info):
        self.bin_x = wh_info["bin"]["x"]
        self.bin_y = wh_info["bin"]["y"]
        self.bin_z = wh_info["bin"]["z"]
        self.bin_d = wh_info["bin"]["pick_distance"]

        self.shelf_x = wh_info["shelf"]["x"] * self.bin_x
        self.shelf_y = wh_info["shelf"]["y"] * self.bin_y
        self.shelf_z = wh_info["shelf"]["z"] * self.bin_z

        self.aisle_x = wh_info["warehouse"]["aisle_x"]
        self.aisle_y = wh_info["warehouse"]["aisle_y"]

        self.count_x = wh_info["warehouse"]["x"]
        self.count_y = wh_info["warehouse"]["y"]

        self.bdr_top = wh_info["border_top"]["padding"]
        self.bdr_down = wh_info["border_down"]["padding"]
        self.bdr_left = wh_info["border_left"]["padding"]
        self.bdr_right = wh_info["border_right"]["padding"]

        self.drop_points_top = wh_info["border_top"]["drop_points"]
        self.drop_points_down = wh_info["border_down"]["drop_points"]
        self.drop_points_left = wh_info["border_left"]["drop_points"]
        self.drop_points_right = wh_info["border_right"]["drop_points"]

    def generate_zones(self):
        # Borders of shelving units
        xlim = (self.shelf_x * self.count_x) + (self.aisle_x * (self.count_x-1))
        ylim = (self.shelf_y * self.count_y) + (self.aisle_y * (self.count_y-1))
        zlim = self.shelf_z

        shelves = []

        # Create shelves: List of small zones used to define individual shelf boundaries
        for x in range(self.count_x):
            x = x * (self.shelf_x + self.aisle_x) + self.bdr_left

            for y in range(self.count_y):
                y = y * (self.shelf_y + self.aisle_y) + self.bdr_top

                shelves.append(Zone(
                    [x, self.shelf_x + x], 
                    [y, self.shelf_y + y], 
                    [0, self.shelf_z]
                    ))

        # Create wh_zone: Range of warehouse floor boundary used to create occupancy map
        wh_zone = Zone(
                [0, xlim + self.bdr_left + self.bdr_right],
                [0, ylim + self.bdr_top  + self.bdr_down],
                [0, zlim]
            )        

        # Create shelf_zone: Range of shelving area boundary used to assign pick points
        shelf_zone = Zone(
                [self.bdr_left, xlim + self.bdr_left],
                [self.bdr_top,  xlim + self.bdr_top],
                [0,             zlim]
            )

        print(f"WAREHOUSE ZONE DIMENSIONS: {wh_zone}")

        return shelves, shelf_zone, wh_zone,
    
    def generate_points(self):
        x_range = self.shelf_zone.x_lims
        y_range = self.shelf_zone.y_lims
        z_range = self.shelf_zone.z_lims

        buffer = self.bin_d

        # Populate pick_xs: a list of possible pick point y values
        # performed by loop through shelf rows to avoid populating isles
        pick_ys = []
        for row in range(self.count_y):
            # Y start of first pick point on shelves
            y_anchor = row * (self.shelf_y+self.aisle_y) + y_range[0]

            pick_ys_row = np.arange(self.bin_y/2, self.shelf_y, self.bin_y) + y_anchor

            pick_ys.extend(list(pick_ys_row))


        # Populate pick_xs: a list of possible pick point x values
        # performed by contatenation of 2 lists for the front and rear side of each shelf
        pick_xs_1 = np.arange(x_range[0], x_range[1], self.shelf_x+self.aisle_x) - buffer
        pick_xs_2 = np.arange(x_range[0], x_range[1], self.shelf_x+self.aisle_x) + buffer + self.shelf_x
        pick_xs = list(np.append(pick_xs_1, pick_xs_2))
        
        # Populate pick_zs
        # Add points in the middle of every bin, but skip the bottom row (AMR's will drive there instead)
        pick_zs = np.arange(self.bin_z/2 + self.bin_z,z_range[1],self.bin_z)

        for k in pick_zs:
            for j in pick_ys:
                for i in pick_xs:
                    self.pick_points.append(Point(i,j,k))

        print(f"TOTAL PICK_POINTS = {len(self.pick_points)}")

        # Populate Drop locations between 1 or all warehouse sides
        if self.drop_points_top:
            self.drop_points.extend(
                [Point(i, 5, 5) for i in np.arange(5, self.wh_zone.x_lims[1], 20)]
            )
        
        if self.drop_points_down:
            self.drop_points.extend(
                [Point(self.wh_zone.x_lims[1]-5, i, 5) for i in np.arange(5, self.wh_zone.x_lims[1], 20)]
            )
        
        if self.drop_points_left:
            self.drop_points.extend(
                [Point(5, i, 5) for i in np.arange(5,self.wh_zone.y_lims[1],20)]
            )

        if self.drop_points_right:
            self.drop_points.extend(
                [Point(i, self.wh_zone.y_lims[1]-5, 5) for i in np.arange(5, self.wh_zone.y_lims[1], 20)]
            )

        return self.pick_points, self.drop_points

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

    def cell_to_point_center(self, cell):
        """
        Converts Point from row-col-layer to x-y-z in feet.

        Args:
            cell (Point): cell to be converted

        Returns:
            Point: point where x, y, and z are in feet
        """
        half_cell = 1/self.resolution /2
        point_x = cell.x/self.resolution + self.wh_zone.x_lims[0] + half_cell
        point_y = cell.y/self.resolution + self.wh_zone.y_lims[0] + half_cell
        point_z = cell.z/self.resolution + self.wh_zone.z_lims[0] + half_cell
        return Point(point_x, point_y, point_z)
    
    def cell_to_point_edge(self, cell):
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
        """
        Coverts a PointZone to a CellZone.
        """
        cell_c1 = self.point_to_cell(point_zone.corners[0])
        cell_c2 = self.point_to_cell(point_zone.corners[1])
        return CellZone([cell_c1.x, cell_c2.x], 
                        [cell_c1.y, cell_c2.y], 
                        [cell_c1.z, cell_c2.z])

    def zone_cells_to_points(self, cell_zone):
        """
        Coverts a CellZone to a PointZone.
        """
        point_c1 = self.cell_to_point_edge(cell_zone.corners[0])
        point_c2 = self.cell_to_point_edge(cell_zone.corners[1])
        return PointZone([point_c1.x, point_c2.x], 
                         [point_c1.y, point_c2.y], 
                         [point_c1.z, point_c2.z])

    def make_occupancy_matrix(self, resolution):
        """
        Returns a 3D occupacy matrix corresponding to this map in the given resolution (cells per foot).
        """
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
        """
        Returns a matrix of 1s (blocked) and 0s (free) of the specified layer of the map (z-level).
        """
        matrix_layer = self.occupancy_matrix[:,:,layer_num].astype(int)
        return(matrix_layer)
    
    def show_occ_matrix(self, layer_num, highlight_cells = []):
        """
        Prints the layer of the occupancy matrix with Xs and •s, with the x axis (and first dimension) in the horizontal direction, y in the vertical direction, with axes labeled by cell number.
        """
        space = "   "
        free = "•"
        blocked = "■"
        matrix_layer = self.occupancy_matrix[:,:,layer_num]
        num_rows = len(self.occupancy_matrix)
        num_cols = len(self.occupancy_matrix[0])

        line0 = "    "
        for x in range(num_rows):
            line0 += str(x).ljust(4)
        lines = [line0]
        for j in range(num_cols):
            line = str(j).ljust(4)
            for i in range(num_rows):
                if self.occupancy_matrix[i,j,layer_num]:
                    addition = blocked.ljust(4)
                else:
                    addition = free.ljust(4)
                if (i,j) in highlight_cells or (i,j,layer_num) in highlight_cells:
                    addition = f"{terminal_colors['FAIL']}{addition}{terminal_colors['ENDC']}"

                line += addition
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