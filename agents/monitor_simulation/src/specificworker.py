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
from numpy.linalg import inv
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
        ##TRANSFORM##
        carla_init = np.array([0, 0])
        carla_fin = np.array([0, 100])
        mundo_init = np.array([-52.43419857889414, 95.23013579559326])
        mundo_fin = np.array([-25.01282223534584, -0.6501019811630249])
        v1 = carla_fin - carla_init
        v2 = mundo_fin -mundo_init
        # self.trans = mundo_init
        self.angle = -np.arccos(np.dot(v1 / np.linalg.norm(v1), v2 / np.linalg.norm(v2)))
        self.R = np.array(np.array([[np.cos(self.angle), np.sin(self.angle), 0],
                                    [-np.sin(self.angle), np.cos(self.angle), 0],
                                    [0, 0, 1]]))
        self.T = np.array(np.array([[1, 0, -mundo_init[0]],
                                    [0, 1, -mundo_init[1]],
                                    [0, 0, 1]]))
        #############
        self.current_time = 0.0
        self.actor_list = []
        self.results = {
            'valid': False,
            'n_simulation': 0,
            'fullResult': [],
            'time': 0
        }
        self.valid_result = []
        self.i = 1
        self.init_time = time.time()
        self.simulation_time = 30.0
        self.variable_simulation_time = float(np.copy(self.simulation_time))
        self.revision_time = 5.0
        self.max_simulation = 5
        self.n_simulation = np.copy(self.max_simulation)
        self.cond_env = {
            'n_simulation': self.max_simulation,
            'duration': self.simulation_time,
            'actorList': []
        }
        self.actor_list = []
        self.get_actor_from_dsr()
        self.load_simulation()
        self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))
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
        self.current_time = time.time() - self.init_time
        print(self.current_time)
        if self.results['valid']:
            if self.current_time >= self.i * self.revision_time:
                print("Processing data")
                self.processing()
                self.add_pose_from_dsr()
                self.i += 1
                self.results['valid'] = False
                self.load_simulation()
                self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))
            elif self.simulation_time - self.current_time < 6.5:
                print("Decision Making")
                self.decision_making()
                self.i = 1
                self.init_time = time.time()
                self.current_time = time.time() - self.init_time
                self.get_actor_from_dsr()
                self.results['valid'] = False
                self.load_simulation()
                self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))
            else:
                print("Waiting to revision time")
        else:
            print('Waiting valid result')
            self.results = self.interface2agent(self.carla_proxy.getState())

        return True

    # def compute(self):
    #     # print('SpecificWorker.compute...')
    #     if self.robot:
    #         self.results = self.interface2agent(self.carla_proxy.getState())
    #         self.current_time = time.time() - self.init_time
    #         print(self.current_time)
    #         print('RESULTADO' ,self.results['valid'], 'n_simulation', self.n_simulation)
    #         if not self.results['valid'] and self.n_simulation > 0:
    #             self.load_simulation()
    #             self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))
    #         else:
    #             if self.current_time >= self.i*self.revision_time:
    #                 print("Processing data")
    #                 self.processing()
    #                 self.add_pose_from_dsr()
    #                 self.i += 1
    #             elif self.simulation_time - self.current_time < 6.5:
    #                 print("Decision Making")
    #                 self.decision_making()
    #                 self.i = 1
    #                 self.init_time = time.time()
    #                 self.get_actor_from_dsr()
    #             else:
    #                 print("Waiting to revision time")
    #     else:
    #         print("There aren't any robot in G")
    #     return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def load_simulation(self):
        self.cond_env = {
            'n_simulation': self.n_simulation,
            'duration': self.simulation_time - self.current_time,
            'actorList': self.actor_list
        }

    def get_actor_from_dsr(self):
        # Read and store people from dsr
            self.actor_list = []
            fullpose = []
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), self.robot.id)
            pos = edge_rt.attrs['rt_translation'].value
            rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
            carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
            pos = [carla_pose[0], carla_pose[1], pos[2]]
            rot[2] = carla_pose[2]
            pose = list(map(float, pos))+list(map(float, rot))
            fullpose.append(pose)
            actor = {
                'id': self.robot.id,
                'carlaID': 0,
                'fullPose': fullpose,
                'rol': 'ego_vehicle'
            }
            self.actor_list.append(actor)
            #self.create_ghost_node(robot, ego)
             # Comprobar cual hay que almacenar y cual no
            vehicles_nodes = self.g.get_nodes_by_type('vehicle')
            for vehicle_node in vehicles_nodes:
                fullpose = []
                edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), vehicle_node.id)
                pos = edge_rt.attrs['rt_translation'].value
                rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
                carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
                pos = [carla_pose[0], carla_pose[1], pos[2]]
                rot[2] = carla_pose[2]
                pose = list(map(float, pos)) + list(map(float, rot))
                fullpose.append(pose)
                actor = {
                    'id': int(vehicle_node.id),
                    'carlaID': 0,
                    'fullPose': fullpose,
                    'rol': 'vehicle'
                }
                self.actor_list.append(actor)
            people_nodes = self.g.get_nodes_by_type('person')
            for person_node in people_nodes:
                fullpose = []
                edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), person_node.id)
                pos = edge_rt.attrs['rt_translation'].value
                rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
                carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
                pos = [carla_pose[0], carla_pose[1], pos[2]]
                rot[2] = carla_pose[2]
                pose = list(map(float, pos)) + list(map(float, rot))
                fullpose.append(pose)
                actor = {
                    'id': int(person_node.id),
                    'carlaID': 0,
                    'fullPose': fullpose,
                    'rol': 'person'
                }
                self.actor_list.append(actor)

    def add_pose_from_dsr(self):
        for actor in self.actor_list:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), actor['id'])
            pos = edge_rt.attrs['rt_translation'].value
            rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
            carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
            pos = [carla_pose[0], carla_pose[1], pos[2]]
            rot[2] = carla_pose[2]
            pose = list(map(float, pos)) + list(map(float, rot))
            actor['fullPose'].append(pose)

    def processing(self):
        fullresult = self.results['fullResult']
        fullresult = fullresult + self.valid_result
        self.valid_result = []
        for result in fullresult:
            error = 0.0
            actor_info = result['actorList']
            # for actor in actor_info:
            for i in range(0, len(actor_info)):
                pose = actor_info[i]['fullPose'][0]
                edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), actor_info[i]['id'])
                x, y, _ = edge_rt.attrs['rt_translation'].value
                carla_pose = self.world_to_carla(x, y, 0)
                x = carla_pose[0]
                y = carla_pose[1]
                error += np.sqrt(np.power(x-pose[0], 2)+np.power(y-pose[1], 2))
                actor_info[i]['fullPose'].pop(0)
            print("ERROR:", error)
            if error <= 10:
                self.valid_result.append(result)
            else:
                print("Simulación erronea")
        # print("Simulaciones válidas: ", len(fullresult))
        self.n_simulation = self.max_simulation - len(self.valid_result)
        print('Simulaciones siguientes', self.n_simulation)

    def decision_making(self):
        print('Decision Making')
        self.n_simulation = self.max_simulation
        print('RESULTADOS VALIDOS', len(self.valid_result))
        for result in self.valid_result:
            virtual_brake = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_brake')
            if virtual_brake is None and result['isBrake']:
                print("Insert is_brake virtual edge")
                brake = Edge(self.robot.id, self.robot.id, 'virtual_brake', self.agent_id)
                self.g.insert_or_assign_edge(brake)
            virtual_collision = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_collision')
            if virtual_collision is None and result['collision']:
                collision = Edge(self.robot.id, self.robot.id, 'virtual_collision', self.agent_id)
                self.g.insert_or_assign_edge(collision)
                print("Insert collision virtual edge")
        self.valid_result = []

    def interface2agent(self, interface_data):
        result = []
        for res in interface_data.fullresult:
            simresult = {
                'collision': res.collision,
                'isBrake': res.isbreak,
                'actorList': []
            }
            for actor in res.actorlist:
                act = {
                    'id': actor.id,
                    'carlaID': actor.carlaid,
                    'fullPose': [],
                    'rol': actor.rol
                }
                for pose in actor.pose:
                    act['fullPose'].append([pose.tx, pose.ty, pose.tz, pose.rx, pose.ry, pose.rz])
                simresult['actorList'].append(act)
            result.append(simresult)
        data = {
            'valid': interface_data.valid,
            'n_simulation': interface_data.nsimulation,
            'time': interface_data.time,
            'fullResult': result
        }
        return data

    def agent2interface(self, cond_env):
        cond_sim = ifaces.RoboCompCarla.Simulations()
        cond_sim.nsimulation = cond_env['n_simulation']
        cond_sim.duration = cond_env['duration']
        cond_sim.actorlist = ifaces.Actors()
        for actor in cond_env['actorList']:
            act = ifaces.RoboCompCarla.Actor()
            act.id = actor['id']
            act.carlaid = actor['carlaID']
            act.rol = actor['rol']
            act.pose = ifaces.Fullposedata()
            for pose in actor['fullPose']:
                pos = ifaces.RoboCompCarla.Posedata(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
                act.pose.append(pos)
            cond_sim.actorlist.append(act)
        return cond_sim

    def world_to_carla(self, x, y, alfa):
        # carla_point = self.R @ (np.array([x/1000, y/1000]) - self.trans).T
        carla_point = self.R @ (self.T @ np.array([x/1000, y/1000, 1.0]).T)
        alfa = -np.rad2deg(alfa)-74
        return [-carla_point[0], carla_point[1], alfa]
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
