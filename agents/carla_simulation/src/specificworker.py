#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
import carla
import cv2
import numpy as np
import time
import random
from rich.console import Console
import interfaces as ifaces
from genericworker import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        self.client = None
        self.world = None
        self._blueprint_library = None
        self.simulation = None
        self.carla_actors = []
        self.collisions = []
        self.results = ifaces.RoboCompCarla.Results()
        self.result.valid = False
        # self.spawn_points = []
        self.INPUT_WIDTH = 424
        self.INPUT_HEIGHT = 240
        self.car_rgb_cams = {}
        # self._server_clock = pygame.time.Clock()
        self.create_client()  # Inicializamos conexión con Carla
        # pygame.init()
        self.initialize_world('CampusV16')  # Cargamos Mapa (En este caso Town01, hay que cambiar por el nuestro)
        # text_file = open("./etc/CampusV16.xodr", "r")
        # self.map = carla.Map("CampusV16", text_file.read())
        # self.load_spawn_points()
        # self.spawn_points = self.world.get_map().get_spawn_points()
        # self.tm = self.client.get_trafficmanager(8000)

    def create_client(self, host=None, port=None):
        if port is None:
            self.port = 2000
        else:
            self.port = port
        if host is None:
            self.host = "localhost"
        else:
            self.host = host
        if self.client is None:
            try:
                self.client = carla.Client(self.host, self.port)
                return True
            except RuntimeError:
                console.log(f"Could not create Carla client on '{self.host}' and {self.port}", style="red")
        else:
            console.log(f"Carla client for {self.port} and {self.host} already exists")
        return False

    def initialize_world(self, map_name):
        if self.client:
            self.client.set_timeout(10.0)
            self.init_time = time.time()
            print('Loading world...')
            self.client.load_world(map_name)
            self.world = self.client.get_world()
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)
            print(f'Loading world took {time.time() - self.init_time:2.2f} seconds')
            self._blueprint_library = self.world.get_blueprint_library()
        else:
            console.log("No carla client created. Call create_client first.", style="red")

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')
        if self.simulation is not None:
            sim = self.simulation.simulations
            actor_list_result = sim.actorlist
            actor_list_result.pose = ifaces.Fullposedata()
            n_simulations = self.simulation.nsimulation
            full_results = ifaces.Fullresults()
            duration = sim.duration
            for _ in range(1, n_simulations):
                self.load_actors(sim.actorlist)
                steep = 0
                while steep < duration:
                    i=1
                    self.simulator.world.tick()
                    if steep == i * self.revision_time:
                        self.add_actor_pose(actor_list_result)
                        i += 1
                    steep += 1
                    self.mosaic()
                        # time_simulation =
        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def load_actors(self, actors):
        for actor in actors:
            print(actor.rol)
            if actor.rol == "ego_vehicle":
                bp = self._blueprint_library.find('vehicle.tesla.model3') #Nuestro vehiculo
                bp.set_attribute('role_name', 'ego')
            elif actor.rol == "vehicle":
                bp = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
            elif actor.rol == "people":
                bp = random.choice(self.world.get_blueprint_library().filter('pederstian.*'))
            else:
                print("Unknown actor")
            try:
                self.spawn_point = carla.Transform(carla.Location(x=actor.pose[0].tx/1000, y=actor.pose[0].ty/1000, z=actor.pose[0].tz/1000 + 385),
                                              carla.Rotation(actor.pose[0].rx, actor.pose[0].ry, actor.pose[0].rz))  # Comprobar angulos CARLA-ROBOCOMP
            except:
                self.spawn_point = random.choice(self.spawn_points) if self.spawn_points else carla.Transform()
                # spawn_point = self.world.get_random_location_from_navigation()
                # print("Error to loaded vehicle")
            # self.world.spawn_actor(bp, spawn_point)
            self.carla_actors.append(self.world.spawn_actor(bp, self.spawn_point))
            print(self.carla_actors)
            actor.carla_id = self.carla_actors[-1].id
        self.set_ego_sensors(self.carla_actors[0], 3)

    def set_ego_sensors(self, ego_vehicle, num_cams):
        cam_bp = self._blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(self.INPUT_HEIGHT))
        cam_bp.set_attribute("image_size_y", str(self.INPUT_WIDTH))
        cam_bp.set_attribute("fov", str(65))
        dimensiones_car = ego_vehicle.bounding_box.extent
        # In carla location is (pitch, yaw, roll) where pitch is y, yaw is z and roll is x

        for i in range(num_cams):
            if i == 0:
                center_cam_location = carla.Location(dimensiones_car.x + 0.02, 0, dimensiones_car.z + 1)
                center_cam_rotation = carla.Rotation(0, 0, 0)
                cam_transform = carla.Transform(center_cam_location, center_cam_rotation)
                self.center_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
                                                         attachment_type=carla.AttachmentType.Rigid)
                self.center_cam.listen(lambda image: self.sensor_rgb_callback(image, "CenterCam"))
            elif i == 1:
                left_cam_location = carla.Location(dimensiones_car.x, -0.02, dimensiones_car.z + 1)
                left_cam_rotation = carla.Rotation(0, -60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                left_cam_transform = carla.Transform(left_cam_location, left_cam_rotation)
                self.left_cam = self.world.spawn_actor(cam_bp, left_cam_transform, attach_to=ego_vehicle,
                                                       attachment_type=carla.AttachmentType.Rigid)
                self.left_cam.listen(lambda image: self.sensor_rgb_callback(image, "LeftCam"))
            elif i == 2:
                right_cam_location = carla.Location(dimensiones_car.x, 0.02, dimensiones_car.z + 1)
                right_cam_rotation = carla.Rotation(0, 60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                right_cam_transform = carla.Transform(right_cam_location, right_cam_rotation)

                self.right_cam = self.world.spawn_actor(cam_bp, right_cam_transform, attach_to=ego_vehicle,
                                                        attachment_type=carla.AttachmentType.Rigid)

                self.right_cam.listen(lambda image: self.sensor_rgb_callback(image, "RightCam"))
        collision_bp = self._blueprint_library.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(collision_bp, cam_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.collision_sensor.listen(lambda collision: self.collision_callback(collision))

    def mosaic(self):
        v_f = cv2.hconcat([self.car_rgb_cams["LeftCam"], self.car_rgb_cams["CenterCam"]])
        v_f = cv2.hconcat([v_f, self.car_rgb_cams["RightCam"]])
        cv2.imshow("FrontCam", v_f)
        cv2.waitKey(1)

    def add_actor_pose(self, actor_list_result):
        for carla_actor in self.carla_actors:

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of get_state method from Carla interface
    #
    def Carla_getState(self):
        if self.Results.valid:
            ret = self.Results
        else:
            ret = None
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of set_simulation_param method from Carla interface
    #
    def Carla_setSimulationParam(self, condini):
        self.simulation = condini
        print(self.simulation)
        pass

    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCarla you can use this types:
    # RoboCompCarla.Posedata
    # RoboCompCarla.Actor
    # RoboCompCarla.Simdata
    # RoboCompCarla.Simulations
    # RoboCompCarla.Simresult
    # RoboCompCarla.Results

