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
        self.robot = None
        self.actor_list = []

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
        self.delete_virtual_rt_edge()
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        self.robot = self.g.get_node('robot')
        if self.robot:
            # self.is_simulation = robot.attrs['simulation'].value
            if self.is_simulation and (not self.loaded):
                self.simulator = Simulation()
                self.actor_list = self.get_actor_from_dsr()
                self.simulator.load_actors(self.actor_list)
                self.loaded = True
                # for v in self.simulator.carla_vehicles:
                #     v.set_autopilot(True, self.simulator.tm_port)
                danger_car = self.simulator.carla_vehicles[1]
                # self.simulator.tm.ignore_lights_percentage(danger_car, 100)
                # self.simulator.tm.distance_to_leading_vehicle(danger_car, 0)
                # self.simulator.tm.vehicle_percentage_speed_difference(danger_car, -20)
            while self.is_simulation:
                self.simulator.world.tick()
                self.simulator.carla_actors[0].apply_control(carla.VehicleControl(brake=0.7))
                self.create_virtual_brake_edge()
                for actor in self.actor_list:
                    print(self.actor_list)
                    self.update_virtual_rt_edges(actor)

                self.simulator.mosaic()
                print('IMAGEN')

            self.delete_virtual_rt_edge()

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    #Place three frontal rgb cams
    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
    #
    def get_actor_from_dsr(self):
        # Read and store people from dsr
        actor_list = []
        edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), self.robot.id)
        x, y, z = edge_rt.attrs['rt_translation'].value
        pos = [x, y, z]
        rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
        rot = [rx, ry, rz]
        ego = ActorType(self.robot.id, None, "ego_vehicle", pos, rot)
        #self.create_ghost_node(robot, ego)
        actor_list.append(ego)
         # Comprobar cual hay que almacenar y cual no
        vehicles_nodes = self.g.get_nodes_by_type('vehicle')
        for vehicle_node in vehicles_nodes:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
            x, y, z = edge_rt.attrs['rt_translation'].value
            pos = [float(x), float(y), float(z)]
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            rot = [float(rx), float(ry), float(rx)]
            vehicle = ActorType(vehicle_node.id, None, "vehicle", pos, rot) #Comprobar cual hay que almacenar y cual no
            actor_list.append(vehicle)
        people_nodes = self.g.get_nodes_by_type('person')
        for person_node in people_nodes:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), person_node.id)
            x, y, z = edge_rt.attrs['rt_translation'].value
            pos = [float(x), float(y), float(z)]
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            rot = [float(rx), float(ry), float(rx)]
            person = ActorType(person_node.id, None, "people", pos, rot)  # Comprobar cual hay que almacenar y cual no
            actor_list.append(person)
        return actor_list

    def update_virtual_rt_edges(self, actor):
        node = self.g.get_node(actor.node_id)
        virtual_edge = self.g.get_edge("world", node.name, "virtual_RT")
        if virtual_edge is None:
            world = self.g.get_node("world")
            virtual_RT = Edge(node.id, world.id, "virtual_RT", self.agent_id)
            virtual_RT.attrs["rt_translation"] = Attribute(actor.pos, self.agent_id)
            virtual_RT.attrs["rt_rotation_euler_xyz"] = Attribute(actor.rot, self.agent_id)
            self.g.insert_or_assign_edge(virtual_RT)
        else:
            actor = self.simulator.world.get_actor(actor.carla_id)
            transform = actor.get_transform()
            pos = [transform.location.x, transform.location.y, transform.location.z]
            rot = [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
            virtual_edge.attrs["rt_translation"] = Attribute(pos, self.agent_id)
            virtual_edge.attrs["rt_rotation_euler_xyz"] = Attribute(rot, self.agent_id)
            self.g.insert_or_assign_edge(virtual_edge)

    def delete_virtual_rt_edge(self):
        edges = self.g.get_edges_by_type("virtual_RT")
        for edge in edges:
            self.g.delete_edge(edge.origin, edge.destination, edge.type)

    def create_virtual_brake_edge(self):
        virtual_brake = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_brake')
        ego_control = self.simulator.carla_actors[0].get_control()
        if virtual_brake is None:
            print(ego_control.brake)
            if ego_control.brake > 0.5:
                time_break = time.time()
                brake = Edge(self.robot.id, self.robot.id, 'virtual_brake', self.agent_id)
                self.g.insert_or_assign_edge(brake)

    def create_virtual_collision_edge(self, collisions):
        for collision in collisions:
            for actor in self.actor_list:
                if actor.carla_id == collision.other_actor.id:
                    if self.g.get_edge(actor.id, self.robot.id, "virtual_collision") is None:
                        collision_edge = Edge(self.robot.id, actor.id, "virtual_collision")
                        self.g.insert_or_assign_edge(collision_edge)
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
