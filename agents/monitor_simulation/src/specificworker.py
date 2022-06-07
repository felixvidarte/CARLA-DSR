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
import interfaces as ifaces
import numpy as np
import time

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
from pydsr import *
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 20
        self.g = DSRGraph(0, "Monitor_simulation", self.agent_id)
        self.rt_api = rt_api(self.g)
        self.robot = self.g.get_node('robot')
        self.current_time = 0.0
        self.actor_list = []
        self.cond_env = None
        self.result = None
        self.fullresult = ifaces.Fullposedata()
        self.process = False
        self.init_time = 0.0
        self.simulation_time = 30.0
        self.variable_simulation_time = float(np.copy(self.simulation_time))
        self.revision_time = 5.0
        self.max_simulation = 5
        self.n_simulation = np.copy(self.max_simulation)
        # try:
        #     signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
        #     signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
        #     signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
        #     signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
        #     signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
        #     signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
        #     console.print("signals connected")
        # except RuntimeError as e:
        #     print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

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
        if self.robot:
            self.current_time = time.time() - self.init_time
            print(self.current_time)
            if not self.process:
                if self.result is None:
                    self.init_time = time.time()
                    self.i = 1
                self.load_simulation()
                self.carla_proxy.setSimulationParam(self.cond_env)
                self.process = True
            else:
                print("Processing simulation data")
            self.result = self.carla_proxy.getState()

            if self.result.valid and self.current_time >= self.i*self.revision_time:
                self.procesing()
                self.i += 1
            else:
                print("Waiting for results")

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

    def load_simulation(self):
        self.actor_list = self.get_actor_from_dsr()
        cond_ini = ifaces.RoboCompCarla.Simdata(self.variable_simulation_time, self.actor_list)
        self.cond_env = ifaces.RoboCompCarla.Simulations(self.n_simulation, cond_ini)

    def get_actor_from_dsr(self):
        # Read and store people from dsr
        actor_list = ifaces.Actors()
        fullpose = ifaces.Fullposedata()
        pose = ifaces.RoboCompCarla.Posedata()
        edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), self.robot.id)
        x, y, z = edge_rt.attrs['rt_translation'].value
        rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
        pose.tx = float(x)
        pose.ty = float(y)
        pose.tz = float(z)
        pose.rx = float(rx)
        pose.ry = float(ry)
        pose.rz = float(rz)
        fullpose.append(pose)
        actor = ifaces.RoboCompCarla.Actor()
        actor.id = self.robot.id
        actor.carlaid = 0
        actor.pose = fullpose
        actor.rol = "ego_vehicle"
        actor_list.append(actor)
        #self.create_ghost_node(robot, ego)
         # Comprobar cual hay que almacenar y cual no
        vehicles_nodes = self.g.get_nodes_by_type('vehicle')
        for vehicle_node in vehicles_nodes:
            fullpose = ifaces.Fullposedata()
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
            pose = ifaces.RoboCompCarla.Posedata()
            x, y, z = edge_rt.attrs['rt_translation'].value
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            pose.tx = float(x)
            pose.ty = float(y)
            pose.tz = float(z)
            pose.rx = float(rx)
            pose.ry = float(ry)
            pose.rz = float(rz)
            fullpose.append(pose)
            actor = ifaces.RoboCompCarla.Actor()
            actor.id = int(vehicle_node.id)
            actor.carlaid = 0
            actor.pose = fullpose
            actor.rol = "vehicle"
            actor_list.append(actor)
        people_nodes = self.g.get_nodes_by_type('person')
        for person_node in people_nodes:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), person_node.id)
            pose = ifaces.RoboCompCarla.Posedata()
            x, y, z = edge_rt.attrs['rt_translation'].value
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            pose.tx = float(x)
            pose.ty = float(y)
            pose.tz = float(z)
            pose.rx = float(rx)
            pose.ry = float(ry)
            pose.rz = float(rz)
            fullpose.append(pose)
            actor = ifaces.RoboCompCarla.Actor()
            actor.id = int(person_node.id)
            actor.carlaid = 0
            actor.pose = fullpose
            actor.rol = "vehicle"
            actor_list.append(actor)
        return actor_list

    def procesing(self):
        print(self.result)
        fullresult = self.result.fullresult
        for i in self.fullresult:
            fullresult.append(i)
        self.fullresult = ifaces.Fullresults()
        for result in fullresult:
            error = 0.0
            actor_info = result.actorlist
            for actor in actor_info:
                pose = actor.pose[0]
                edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), actor.id)
                x, y, _ = edge_rt.attrs['rt_translation'].value
                error += np.sqrt(np.power(x/1000-pose.tx, 2)+np.power(y/1000-pose.ty,2))
                actor.pose.pop(0)
            print("ERROR:", error)
            if error <= 25:
                self.fullresult.append(result)
            else:
                print("Simulación erronea")
            print(len(self.fullresult))
        # print("Simulaciones válidas: ", len(fullresult))
        self.n_simulation = self.max_simulation - len(self.fullresult)
        self.variable_simulation_time = self.simulation_time - self.current_time
        # print("Time", self.variable_simulation_time)
        self.process = False

    ######################
    # From the Carlasim you can call this methods:
    # self.carla_proxy.getstate(...)
    # self.carla_proxy.setsimulationparam(...)

    ######################
    # From the Carlasim you can use this types:
    # Carlasim.Posedata
    # Carlasim.Actor
    # Carlasim.Simdata
    # Carlasim.Simulations
    # Carlasim.Simresult
    # Carlasim.Results



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
