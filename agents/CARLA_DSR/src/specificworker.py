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

import numpy as np
import carla
from PySide2.QtCore import QTimer, Signal, QPoint, QPointF
from PySide2.QtGui import QPolygon, Qt
from PySide2.QtWidgets import QApplication
from genericworker import *

from PersonalSpacesManager import PersonalSpacesManager

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
from pydsr import *

from rich.console import Console

console = Console(highlight=False)


# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class VehicleType:
    def __init__(self, id=None, tx=0, ty=0, ry=0):
        self.id = id
        self.tx = tx
        self.ty = ty
        self.ry = ry


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

        self.agent_id = 121
        self.g = DSRGraph(0, "CARLA_DSR", self.agent_id)
        self.rt_api = rt_api(self.g)

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
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):

        # PEOPLE
        vehicle_list = self.get_vehicles_from_dsr()

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def get_vehicles_from_dsr(self):
        # Read and store people from dsr
        vehicles_nodes = self.g.get_nodes_by_type('vehicle')
        vehicles_list = []
        for vehicle_node in vehicles_nodes:
            vehicle_id = vehicle_node.attrs['vehicle_id'].value
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
            tx, ty, tz = edge_rt.attrs['rt_translation'].value
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            vehicle_type = VehicleType(vehicle_id, tx, ty, ry)
            vehicles_list.append(vehicle_type)

        return vehicles_list


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
