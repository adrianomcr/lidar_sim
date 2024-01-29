#!/usr/bin/env python3
# -*- coding:utf-8 -*-


import json
import os

# def set_parameter(key, value):
#     parameters = load_parameters()
#     parameters[key] = value
#     save_parameters(parameters)

# def get_parameter(key):
#     parameters = load_parameters()
#     return parameters.get(key)

# def load_parameters():
#     try:
#         with open(PARAM_FILE, "r") as file:
#             return json.load(file)
#     except FileNotFoundError:
#         return {}

# def save_parameters(parameters):
#     with open(PARAM_FILE, "w") as file:
#         json.dump(parameters, file, indent=4)

# PARAM_FILE = os.path.expanduser('~')+"/simulation_ws/src/px4sim/config/sim_params.json"



# planes.push_back( new plane_class(1,0,0, 5));


class json_generator:
    """
    Class that ...
    """

    def __init__(self):
        
        self.data = {}

        self.plane_count = 0
        self.sphere_count = 0
        self.cylinder_count = 0
        self.ellipse_count = 0

    def add_plane(self, center, axis, description=""):
        self.plane_count = self.plane_count+1
        d = {
        'description': description,
        'type': 'plane',
        'center': center,
        'axis': axis,
        'radius': [],
        'size': [],
        'rotation': []
        }
        self.data["PLANE_"+str(self.plane_count).zfill(4)] = d

    def add_sphere(self, center, radius, description=""):
        self.sphere_count = self.sphere_count+1
        d = {
        'description': description,
        'type': 'sphere',
        'center': center,
        'axis': [],
        'radius': radius,
        'size': [],
        'rotation': []
        }
        self.data["SPHERE_"+str(self.sphere_count).zfill(4)] = d

    def add_cylinder(self, center, axis, radius, bounds=[], description=""):
        self.cylinder_count = self.cylinder_count+1
        d = {
        'description': description,
        'type': 'cylinder',
        'center': center,
        'axis': axis,
        'radius': radius,
        'size': [],
        'rotation': [],
        'bounds': bounds
        }
        self.data["CYLINDER_"+str(self.cylinder_count).zfill(4)] = d

    def add_ellipsoid(self, center, size, description=""):
        self.ellipse_count = self.ellipse_count+1
        d = {
        'description': description,
        'type': 'ellipsoid',
        'center': center,
        'axis': [],
        'radius': [],
        'size': size,
        'rotation': []
        }
        self.data["ELLIPSOID_"+str(self.ellipse_count).zfill(4)] = d


    def save_parameters(self,file_path):
        with open(file_path, "w") as file:
            json.dump(self.data, file, indent=2)


if __name__ == "__main__":

    params = json_generator()

    # Planes
    params.add_plane(center=[0,0,0], axis=[0,0,1], description='Hangar main floor')
    params.add_plane(center=[5,0,0], axis=[1,0,0], description='Hangar wall')
    params.add_plane(center=[-65,0,0], axis=[1,0,0], description='Hangar wall')
    params.add_plane(center=[0,30,0], axis=[0,1,0], description='Hangar wall')
    params.add_plane(center=[0,-30,0], axis=[0,1,0], description='Hangar wall')
    params.add_plane(center=[0,0,25], axis=[0,0.3,1], description='Hangar ceiling')
    params.add_plane(center=[0,0,25], axis=[0,-0.3,1], description='Hangar ceiling')

    # Cylinders
    params.add_cylinder(center=[-10,0,25], axis=[0,1,0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-20,0,25], axis=[0,1,0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-30,0,25], axis=[0,1,0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-40,0,25], axis=[0,1,0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-50,0,25], axis=[0,1,0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-10,0,25], axis=[0,1,-0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-20,0,25], axis=[0,1,-0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-30,0,25], axis=[0,1,-0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-40,0,25], axis=[0,1,-0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-50,0,25], axis=[0,1,-0.3], radius=0.4, description='Hangar ceiling structure')
    params.add_cylinder(center=[-10,30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-20,30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-30,30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-40,30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-50,30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-10,-30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-20,-30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-30,-30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-40,-30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-50,-30,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[5,0,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[-65,0,0], axis=[0,0,1], radius=0.4, description='Hangar vertical wall structure')
    params.add_cylinder(center=[0,30,7], axis=[1,0,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[0,30,14], axis=[1,0,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[0,-30,7], axis=[1,0,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[0,-30,17], axis=[1,0,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[5,0,7], axis=[0,1,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[5,0,14], axis=[0,1,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[-65,0,7], axis=[0,1,0], radius=0.4, description='Hangar horizontal wall structure')
    params.add_cylinder(center=[-65,0,14], axis=[0,1,0], radius=0.4, description='Hangar horizontal wall structure')

    # Spheres
    params.add_sphere(center=[-5,10,0], radius=2, description='A test sphere')




    # Airplane cylinders
    params.add_cylinder(center=[-23,8,4.5], axis=[1,0,0], radius=1.5, bounds=[-1,4], description='Airplane left internal engine')
    params.add_cylinder(center=[-23,-8,4.5], axis=[1,0,0], radius=1.5, bounds=[-1,4], description='Airplane right internal engine')
    params.add_cylinder(center=[-24,15,4.5], axis=[1,0,0], radius=1.5, bounds=[-1,4], description='Airplane left external engine')
    params.add_cylinder(center=[-24,-15,4.5], axis=[1,0,0], radius=1.5, bounds=[-1,4], description='Airplane right external engine')
    # Airplane ellipsoids
    params.add_ellipsoid(center=[-30,0,5], size=[28,4,4], description='Airplane main fuselage')
    params.add_ellipsoid(center=[-27,0,6], size=[5,25,0.5], description='Airplane main wings')
    params.add_ellipsoid(center=[-52,0,11], size=[2.5,0.5,8], description='Airplane tail')
    params.add_ellipsoid(center=[-52,0,16], size=[4,8,0.6], description='Airplane tail wing')





    params.save_parameters('../config/world.json')