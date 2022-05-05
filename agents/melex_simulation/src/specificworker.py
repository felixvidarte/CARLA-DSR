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
import carla
import weakref
from PySide2.QtCore import QTimer, Signal, QPoint, QPointF
from PySide2.QtGui import QPolygon, Qt
from PySide2.QtWidgets import QApplication
from genericworker import *
from threading import Lock
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
from pydsr import *
from simulation import *
from rich.console import Console

console = Console(highlight=False)

# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        # pygame.init()
        # pygame.font.init()
        self.agent_id = 121
        self.g = DSRGraph(0, "CARLA_DSR", self.agent_id)
        self.rt_api = rt_api(self.g)
        self.simulator = None
        self.is_simulation = True
        self.loaded = False
        self.vehicle_list = []
        self.ghost_nodes = []

        # try:
        #     signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
        #     signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
        #     signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
        #     signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
        #     signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
        #     signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
        #     print("signals connected")
        # except RuntimeError as e:
        #     print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        self.delete_ghost_node()
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        robot = self.g.get_node('robot')
        if robot:
            # self.is_simulation = robot.attrs['simulation'].value
            if self.is_simulation and (not self.loaded):
                print(self.loaded)
                self.simulator = Simulation()
                self.vehicle_list = self.get_vehicles_from_dsr()
                self.simulator.load_vehicles(self.vehicle_list)
                self.loaded = True
            while self.is_simulation:
                self.simulator.world.tick()
                self.simulator.vehicles[0].apply_control(carla.VehicleControl(throttle=0.3))
                for vehicle in self.vehicle_list:
                    self.create_or_update_virtual_RT_edges(vehicle)
                print(self.simulator.vehicles)
                self.simulator.mosaic()


                print('IMAGEN')
            self.delete_ghost_node()


        # if self.i == 0:
        #     self.world.on_tick(self.on_world_tick)
        #     self.i = 1
        # cv2.imshow("CenterCam", self.car_rgb_cams["CenterCam"])
        # cv2.waitKey(1)
        # cv2.imshow("LeftCam", self.car_rgb_cams["LeftCam"])
        # cv2.waitKey(1)
        print("HOLA")

        # imagecompuesta = self.mosaic()

        #ego_cam.listen(lambda image: image.save_to_disk('/home/salabeta/TFGFelix/CARLA-DSR/agents/CARLA_DSR/%.6d.png' % image.frame))
        # PEOPLE
        # vehicle_list = self.get_vehicles_from_dsr()
        # for vehicle in vehicle_list:
        #     self.load_vehicle(vehicle)
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    #Place three frontal rgb cams
    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
    #
    def get_vehicles_from_dsr(self):
        # Read and store people from dsr
        vehicles_list = []
        robot = self.g.get_node('robot')
        edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), robot.id)
        x, y, z = edge_rt.attrs['rt_translation'].value
        pos = [x, y, z]
        rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
        rot = [rx, ry, rz]
        ego = VehicleType(robot.id, pos, rot)
        print(ego.pos)
        #self.create_ghost_node(robot, ego)
        vehicles_list.append(ego)
         # Comprobar cual hay que almacenar y cual no
        vehicles_nodes = self.g.get_nodes_by_type('vehicle')
        for vehicle_node in vehicles_nodes:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
            x, y, z = edge_rt.attrs['rt_translation'].value
            pos = [float(x), float(y), float(z)]
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            rot = [float(rx), float(ry), float(rx)]
            vehicle = VehicleType(vehicle_node.id, pos, rot) #Comprobar cual hay que almacenar y cual no
            #self.create_ghost_node(vehicle_node, vehicle)
            vehicles_list.append(vehicle)
        return vehicles_list

    def create_or_update_virtual_RT_edges(self, vehicle_type):
        print(vehicle_type.id, vehicle_type.pos)
        node = self.g.get_node(vehicle_type.id)
        print(node.name)
        virtual_edge = self.g.get_edge("world", node.name, "virtual_RT")
        print(virtual_edge)
        if virtual_edge is None:
            world = self.g.get_node("world")
            virtual_RT = Edge(node.id, world.id, "virtual_RT", self.agent_id)
            print(Attribute(vehicle_type.pos, self.agent_id))
            virtual_RT.attrs["rt_translation"] = Attribute(vehicle_type.pos, self.agent_id)
            virtual_RT.attrs["rt_rotation_euler_xyz"] = Attribute(vehicle_type.rot, self.agent_id)
            self.g.insert_or_assign_edge(virtual_RT)
        else:
            virtual_edge.attrs["rt_translation"] = Attribute(vehicle_type.pos, self.agent_id)
            virtual_edge.attrs["rt_rotation_euler_xyz"] = Attribute(vehicle_type.rot, self.agent_id)
            self.g.insert_or_assign_edge(virtual_edge)

    def create_ghost_node(self, vehicle_node, vehicle_type):
        node_name = vehicle_node.name + '_ghost'
        new_node = Node(agent_id=self.agent_id, type=vehicle_node.type, name=node_name)
        new_node.attrs['pos_x'] = Attribute(vehicle_node.attrs['pos_x'].value+25, self.agent_id)
        new_node.attrs['pos_y'] = Attribute(vehicle_node.attrs['pos_y'].value+25, self.agent_id)
        try:
            self.g.insert_node(new_node)
            self.ghost_nodes.append(new_node)
            console.print('Ghost node created -- ', new_node.id, style='red')
            has_edge = Edge(new_node.id, vehicle_node.id, 'has', self.agent_id)
            self.g.insert_or_assign_edge(has_edge)
            self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), new_node.id, vehicle_type.pos, vehicle_type.rot)
            print(' inserted new node  ', vehicle_type.id)

        except:
            traceback.print_exc()
            print('cant insert node or add edge RT')

    def delete_ghost_node(self,):
        for node in self.ghost_nodes:
            for edge in node.edges:
                self.g.delete_edge(edge.origin, edge.destination, edge.type)
            self.g.delete_node(node.id)
        self.ghost_nodes = []

    # def update_RT_edge(self):


    # ===================================================================
    # ===================================================================

    # =============== DSR Methods  ================
    # =============================================

    # def update_node_att(self, id: int, attribute_names: [str]):
    #     console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
    #
    def update_node(self, id: int, type: str):
        if id == 200:
            self.is_simulation = self.g.get_node('robot').attrs['simulation'].value
        else:
            console.print(f"UPDATE NODE: {id} {type}", style='green')

    # def delete_node(self, id: int):
    #     console.print(f"DELETE NODE:: {id} ", style='green')
    #
    # def update_edge(self, fr: int, to: int, type: str):
    #     console.print(f"UPDATE EDGE: {fr} to {to} {type}", style='green')
    #
    # def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
    #     console.print(f"UPDATE EDGE ATT: {fr} to {to} {attribute_names}", style='green')
    #
    # def delete_edge(self, fr: int, to: int, type: str):
    #     console.print(f"DELETE EDGE: {fr} to {to} {type}", style='green')

    ######################
    # From the RoboCompHumanToDSRPub you can use this types:
    # RoboCompHumanToDSRPub.TJointData
    # RoboCompHumanToDSRPub.Person
    # RoboCompHumanToDSRPub.PeopleData
