#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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
import itertools
import traceback
#import pygame
import math
import numpy as np
import carla
import random
import time
import cv2
import pygame
import weakref
from PySide2.QtCore import QTimer, Signal, QPoint, QPointF
from PySide2.QtGui import QPolygon, Qt
from PySide2.QtWidgets import QApplication
from genericworker import *
from threading import Lock

from PersonalSpacesManager import PersonalSpacesManager

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
from pydsr import *

from rich.console import Console

console = Console(highlight=False)

mutex = Lock()
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class VehicleType:
    def __init__(self, id=None, tx=0, ty=0, tz = 0, rx = 0, ry=0, rz = 0):
        self.id = id
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.ry = ry
        self.rx = rx
        self.rz = rz


class ObjectType:
    def __init__(self, id=None, node_id=None, tx=0, ty=0, ry=0,
                 shape=None, inter_angle=0, inter_space=0, depth=0, width=0, height=0):
        self.id = id
        self.node_id = node_id
        self.tx = tx
        self.ty = ty
        self.ry = ry
        self.shape = shape
        self.inter_angle = inter_angle
        self.inter_space = inter_space
        self.depth = depth
        self.width = width
        self.height = height


class SpecificWorker(GenericWorker):

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 500
        # pygame.init()
        # pygame.font.init()
        self.agent_id = 121
        self.g = DSRGraph(0, "CARLA_DSR", self.agent_id)
        self.rt_api = rt_api(self.g)
        self.INPUT_WIDTH = 424
        self.INPUT_HEIGHT = 240
        self.car_rgb_cams = {}
        self._server_clock = pygame.time.Clock()
        self.client = None
        self._blueprint_library = None
        self._vehicle = None
        self.ego_vehicle = None
        self.create_client() #Inicializamos conexión con Carla
        self.client.set_timeout(2.0)
        self.initialize_world('Town01') #Cargamos Mapa (En este caso Town01, hay que cambiar por el nuestro)
        self.world.wait_for_tick()
        self.load_ego_vehicle()
        self.set_ego_rgb_frontal_cams(3)
        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            print("signals connected")
        except RuntimeError as e:
            print(e)

        self.personal_spaces_manager = PersonalSpacesManager()

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        if self.world is not None:
            self.world.destroy()
        self.center_cam.destroy()
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        # cv2.imshow("CenterCam", self.car_rgb_cams["CenterCam"])
        # cv2.waitKey(1)
        # cv2.imshow("LeftCam", self.car_rgb_cams["LeftCam"])
        # cv2.waitKey(1)
        v_f = cv2.hconcat([self.car_rgb_cams["LeftCam"], self.car_rgb_cams["CenterCam"]])
        v_f = cv2.hconcat([v_f, self.car_rgb_cams["RightCam"]])
        cv2.imshow("FrontCam", v_f)
        cv2.waitKey(1)
        #ego_cam.listen(lambda image: image.save_to_disk('/home/salabeta/TFGFelix/CARLA-DSR/agents/CARLA_DSR/%.6d.png' % image.frame))
        # PEOPLE
        # vehicle_list = self.get_vehicles_from_dsr()
        # for vehicle in vehicle_list:
        #     self.load_vehicle(vehicle)
        return True


    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

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
            init_time = time.time()
            print('Loading world...')
            self.world = self.client.load_world(map_name)
            print('Done')
            print(f'Loading world took {time.time() - init_time:2.2f} seconds')
            self._blueprint_library = self.world.get_blueprint_library()
        else:
            console.log("No carla client created. Call create_client first.", style="red")




    def load_vehicle(self, vehicle):
        choices = self.world.get_blueprint_library().filter('vehicle.*')
        vehicle_blueprint = random.choice(choices)
        if self._vehicle is not None:
            spawn_point = self.vehicle.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self._vehicle = self.world.try_spawn_actor(vehicle_blueprint, spawn_point)

        while self._vehicle is None:
            try:
                spawn_point = carla.Transform(carla.Location(x=vehicle.tx, y=vehicle.ty, z=vehicle.tz + 2), carla.Rotation(0, 0, vehicle.rz)) #Comprobar angulos CARLA-ROBOCOMP
                self._vehicle = self.world.spawn_actor(vehicle_blueprint, spawn_point)
            except:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                self._vehicle = self.world.try_spawn_actor(vehicle_blueprint, spawn_point)

    def load_ego_vehicle(self): #Esta la podemos borrar después, es para probar con un solo coche
        ego_bp = self._blueprint_library.find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        self.ego_vehicle = self.world.try_spawn_actor(ego_bp, spawn_point)
        self.world.on_tick(self.on_world_tick)



    def get_vehicles_from_dsr(self):
        # Read and store people from dsr
        vehicles_nodes = self.g.get_nodes_by_type('vehicle')
        vehicles_list = []
        for vehicle_node in vehicles_nodes:
            vehicle_id = vehicle_node.attrs['vehicle_id'].value
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
            tx, ty, tz = edge_rt.attrs['rt_translation'].value
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            vehicle_type = VehicleType(vehicle_id, tx, ty, tz, rx, ry, rz) #Comprobar cual hay que almacenar y cual no
            vehicles_list.append(vehicle_type)

        return vehicles_list

    #Place three frontal rgb cams
    def set_ego_rgb_frontal_cams(self, num_cams):
        cam_bp = self._blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(self.INPUT_HEIGHT))
        cam_bp.set_attribute("image_size_y", str(self.INPUT_WIDTH))
        cam_bp.set_attribute("fov", str(105))
        dimensiones_Car = self.ego_vehicle.bounding_box.extent
        #In carla location is (pitch, yaw, roll) where pitch is y, yaw is z and roll is x

        for i in range(num_cams):
            if i == 0:
                center_cam_location = carla.Location(dimensiones_Car.x, 0, dimensiones_Car.z)
                center_cam_rotation = carla.Rotation(13, 0, 0)
                cam_transform = carla.Transform(center_cam_location, center_cam_rotation)
                self.center_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.ego_vehicle,
                                            attachment_type=carla.AttachmentType.Rigid)
                self.center_cam.listen(lambda image: self.sensor_callback(image, "CenterCam"))
            elif i == 1:
                left_cam_location = carla.Location(dimensiones_Car.x, 0, dimensiones_Car.z)
                left_cam_rotation = carla.Rotation(13, -60, 0)
                #left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                left_cam_transform = carla.Transform(left_cam_location, left_cam_rotation)
                self.left_cam = self.world.spawn_actor(cam_bp, left_cam_transform, attach_to=self.ego_vehicle,
                                                       attachment_type=carla.AttachmentType.Rigid)
                self.left_cam.listen(lambda image: self.sensor_callback(image, "LeftCam"))
            elif i == 2:
                right_cam_location = carla.Location(dimensiones_Car.x, 0, dimensiones_Car.z)
                right_cam_rotation = carla.Rotation(13, 60, 0)
                #left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                right_cam_transform = carla.Transform(right_cam_location, right_cam_rotation)
                self.right_cam = self.world.spawn_actor(cam_bp, right_cam_transform, attach_to=self.ego_vehicle,
                                                       attachment_type=carla.AttachmentType.Rigid)
                self.right_cam.listen(lambda image: self.sensor_rgbcallback(image, "RightCam"))


    #Compute np array for rgb sensors
    def sensor_rgb_callback(self, img, sensorID):
        global mutex
        mutex.acquire()
        array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (self.INPUT_WIDTH, self.INPUT_HEIGHT, 4))
        array = array[:, :, :3]
        self.car_rgb_cams[sensorID] = array
        mutex.release()



    #Dudas de que hace
    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        if self.server_fps in [float("-inf"), float("inf")]:
            self.server_fps = -1
        print('Server FPS', int(self.server_fps))
    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
    #
    def HumanToDSRPub_newPeopleData(self, people):
        print('HumanToDSRPub_newPeopleData ------')

        people_list = people.peoplelist
        people_nodes = self.g.get_nodes_by_type('person')

        for person in people_list:
            person_node_in_dsr = None

            for p_node in people_nodes:
                if p_node.attrs['person_id'].value == person.id:
                    person_node_in_dsr = p_node
                    break

            # Update Node
            if person_node_in_dsr is not None:

                try:
                    self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), person_node_in_dsr.id,
                                                         [person.x, person.y, person.z], [.0, person.ry, .0])
                except:
                    traceback.print_exc()
                    print('Cant update RT edge')

                # print(self.rt_api.get_edge_RT(self.g.get_node("world"), person_node_in_dsr.id))

            # Create Node
            else:

                node_name = 'person_' + str(person.id)
                new_node = Node(agent_id=self.agent_id, type='person', name=node_name)
                new_node.attrs['person_id'] = Attribute(person.id, self.agent_id)
                new_node.attrs['pos_x'] = Attribute(25.0, self.agent_id)
                new_node.attrs['pos_y'] = Attribute(50.0, self.agent_id)

                try:
                    id_result = self.g.insert_node(new_node)
                    self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), id_result,
                                                         [person.x, person.y, person.z], [.0, person.ry, .0])
                    print(' inserted new node  ', id_result)

                except:
                    traceback.print_exc()
                    print('cant update node or add edge RT')

    # ===================================================================
    # ===================================================================

    # =============== DSR Methods  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {to} {type}", style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {to} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {to} {type}", style='green')

    ######################
    # From the RoboCompHumanToDSRPub you can use this types:
    # RoboCompHumanToDSRPub.TJointData
    # RoboCompHumanToDSRPub.Person
    # RoboCompHumanToDSRPub.PeopleData
