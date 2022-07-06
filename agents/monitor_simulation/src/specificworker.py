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
            'fullResult': [],
            'time': 0
        }
        self.valid_result = []
        self.i = 1
        self.init_time = time.time()
        self.simulation_time = 30.0
        self.variable_simulation_time = float(np.copy(self.simulation_time))
        self.revision_time = 3
        self.indice = 0
        # self.max_simulation = 5
        # self.n_simulation = np.copy(self.max_simulation)
        self.cond_env = {
            'indice': self.indice,
            'duration': self.simulation_time,
            'actorList': []
        }
        self.actor_list = []
        self.actor_id = 0
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
        # print(self.current_time)
        # if self.simulation_time - self.current_time < 4:
        #     print("Decision Making")
        #     self.decision_making()
        #     self.i = 1
        #     self.init_time = time.time()
        #     self.current_time = time.time() - self.init_time
        #     self.get_actor_from_dsr()
        #     self.results['valid'] = False
        #     self.load_simulation()
        #     self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))

        if self.results['valid']:
            if self.current_time >= self.i * self.revision_time:
                print("Processing data", self.current_time)
                self.robot = self.g.get_node('robot')
                # print("Velocidad", self.robot.attrs['robot_ref_adv_speed'])
                self.processing()
                self.decision_making()
                self.add_pose_from_dsr()
                self.i += 1
                self.indice += 1
                self.results['valid'] = False
                self.load_simulation()
                self.carla_proxy.setSimulationParam(self.agent2interface(self.cond_env))
            # else:
                # print("Waiting to revision time")
        else:
            # print('Waiting valid result')
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
        # print(self.actor_list)
        self.cond_env = {
            'indice': self.indice,
            'duration': self.simulation_time,
            'actorList': self.actor_list
        }
        # print("Send new simulations", self.cond_env)

    def get_actor_from_dsr(self):
        # Read and store people from dsr
            self.actor_list = []
            fullpose = []
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), self.robot.id)
            pos = edge_rt.attrs['rt_translation'].value
            rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
            carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
            # pos = [carla_pose[0], carla_pose[1], pos[2]]
            # rot[2] = carla_pose[2]
            # pose = list(map(float, pos))+list(map(float, rot))
            fullpose.append(carla_pose)
            actor = {
                'id': self.robot.id,
                'carlaID': 0,
                'initPose': fullpose,
                'fullPose': [],
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
                # pos = [carla_pose[0], carla_pose[1], pos[2]]
                # rot[2] = carla_pose[2]
                # pose = list(map(float, pos)) + list(map(float, rot))
                fullpose.append(carla_pose)
                actor = {
                    'id': vehicle_node.id,
                    'carlaID': 0,
                    'initPose': fullpose,
                    'fullPose': [],
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
                # pos = [carla_pose[0], carla_pose[1], pos[2]]
                # rot[2] = carla_pose[2]
                # pose = list(map(float, pos)) + list(map(float, rot))
                fullpose.append(carla_pose)
                actor = {
                    'id': person_node.id,
                    'carlaID': 0,
                    'initPose': fullpose,
                    'fullPose': [],
                    'rol': 'person'
                }
                self.actor_list.append(actor)

    def add_pose_from_dsr(self):
        for actor in self.actor_list:
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node('world'), actor['id'])
            pos = edge_rt.attrs['rt_translation'].value
            rot = edge_rt.attrs['rt_rotation_euler_xyz'].value
            carla_pose = self.world_to_carla(pos[0], pos[1], rot[2])
            # pos = [carla_pose[0], carla_pose[1], pos[2]]
            # rot[2] = carla_pose[2]
            # pose = list(map(float, pos)) + list(map(float, rot))
            if len(actor['initPose']) > 4:
                actor['initPose'].pop(0)
            actor['initPose'].append(carla_pose)

    def processing(self):
        fullresult = self.results['fullResult']
        # print(fullresult)
        fullresult = fullresult + self.valid_result
        self.valid_result = []
        for result in fullresult:
            total_error = 0.0
            actor_info = result['actorList']
            correct = 0
            # for actor in actor_info:
            for i in range(0, len(actor_info)):
                if (len(actor_info[i]['fullPose'])>0):
                    pose = actor_info[i]['fullPose'][0]
                    edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), actor_info[i]['id'])
                    x, y, _ = edge_rt.attrs['rt_translation'].value
                    carla_pose = self.world_to_carla(x, y, 0)
                    xc = carla_pose[0]
                    yc = carla_pose[1]
                    error = np.sqrt(np.power(xc-pose[0], 2)+np.power(yc-pose[1], 2))
                    # print(actor_info[i]['id'], error)
                    total_error += error
                    if actor_info[i]['rol'] == 'person':
                        if error <= 7:
                            correct += 1
                        # else:
                        #     print("Incorrect person simulation", error)
                    else:
                        if error <= 5:
                            correct += 1
                        # else:
                            # print("Incorrect vehicle simulation", error)
                    actor_info[i]['fullPose'].pop(0)
                print("Total ERROR:", total_error)
                if correct == len(actor_info):
                    self.valid_result.append(result)
                else:
                    print("No position avaliable")
            # else:
                # print(i, "Simulación erronea en ", len(actor_info)-correct, "de ", len(actor_info))
        # print("Simulaciones válidas: ", len(fullresult))

    def decision_making(self):
        print("Resultados validos", len(self.valid_result))
        collision_cont = 0
        brake_cont = 0
        collision_time = []
        for res in self.valid_result:
            if res['collision']['iscollision']:
                self.actor_id = res['collision']['actorcollision']
                time_collision = res['collision']['timecollision']-(self.indice - res['indice'])*self.revision_time
                if time_collision > 0:
                    collision_cont += 1
                    collision_time.append(time_collision)
            if res['isBrake']:
                brake_cont += 1
        if (len(self.valid_result)==0):
            print("Collision", 0, collision_time)
            print("Brake", 0)
        else:
            print("Collision", collision_cont/len(self.valid_result), collision_time)
            print("Brake",brake_cont/len(self.valid_result))
        if collision_cont > 0:
            virtual_collision = self.g.get_edge(self.robot.id, self.actor_id, 'virtual_collision')
            if virtual_collision is None:
                collision = Edge(self.robot.id, self.actor_id, 'virtual_collision', self.agent_id)
                collision.attrs['collision'] = Attribute(collision_cont/len(self.valid_result), self.agent_id)
                collision.attrs['time_collision'] = Attribute(collision_time, self.agent_id)
                self.g.insert_or_assign_edge(collision)
            else:
                virtual_collision.attrs['collision'] = Attribute(collision_cont/len(self.valid_result), self.agent_id)
                virtual_collision.attrs['time_collision'] = Attribute(collision_time, self.agent_id)
                self.g.insert_or_assign_edge(virtual_collision)
        else:
            self.g.delete_edge(self.actor_id, self.robot.id, 'virtual_collision')

        if brake_cont > 0:
            virtual_brake = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_brake')
            if virtual_brake is None:
                brake = Edge(self.robot.id, self.robot.id, 'virtual_brake', self.agent_id)
                brake.attrs['brake'] = Attribute(brake_cont / len(self.valid_result), self.agent_id)
                self.g.insert_or_assign_edge(brake)
            else:
                virtual_brake.attrs['brake'] = Attribute(brake_cont / len(self.valid_result), self.agent_id)
                self.g.insert_or_assign_edge(virtual_brake)
        else:
            self.g.delete_edge(self.robot.id, self.robot.id, 'virtual_brake')
        # self.indice = 0
        # print('RESULTADOS VALIDOS', len(self.valid_result))
        # print(self.valid_result)
        # for result in self.valid_result:
        #     virtual_brake = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_brake')
        #     if virtual_brake is None and result['isBrake']:
        #         print("Insert is_brake virtual edge")
        #         brake = Edge(self.robot.id, self.robot.id, 'virtual_brake', self.agent_id)
        #         self.g.insert_or_assign_edge(brake)
        #     # virtual_collision = self.g.get_edge(self.robot.id, self.robot.id, 'virtual_collision')
        #     # print(virtual_collision)
        #     if result['collision']['iscollision']:
        #         collision = Edge(self.robot.id,result['collision']['actorcollision'], 'virtual_collision', self.agent_id)
        #         self.g.insert_or_assign_edge(collision)
        #         print("Insert collision virtual edge")
        # self.valid_result = []

    def interface2agent(self, interface_data):
        result = []
        for res in interface_data.fullresult:
            simresult = {
                'indice': res.indice,
                'collision': {
                    'iscollision': res.collision.iscollision,
                    'timecollision': res.collision.timecollision,
                    'actorcollision': res.collision.actorcollision
                },
                'isBrake': res.isbrake,
                'actorList': []
            }
            for actor in res.actorlist:
                act = {
                    'id': actor.id,
                    'carlaID': actor.carlaid,
                    'initPose': [],
                    'fullPose': [],
                    'rol': actor.rol
                }
                for pose in actor.pose:
                    act['fullPose'].append([pose.tx, pose.ty, pose.rz])
                simresult['actorList'].append(act)
            result.append(simresult)
        data = {
            'valid': interface_data.valid,
            'time': interface_data.time,
            'fullResult': result
        }
        return data

    def agent2interface(self, cond_env):
        cond_sim = ifaces.RoboCompCarla.Simulations()
        cond_sim.indice = cond_env['indice']
        cond_sim.duration = cond_env['duration']
        cond_sim.actorlist = ifaces.Actors()
        for actor in cond_env['actorList']:
            act = ifaces.RoboCompCarla.Actor()
            act.id = actor['id']
            act.carlaid = actor['carlaID']
            act.rol = actor['rol']
            act.initpose = ifaces.Fullposedata()
            act.pose = ifaces.Fullposedata()
            for pose in actor['initPose']:
                pos = ifaces.RoboCompCarla.Posedata(pose[0], pose[1], 0, 0, 0, pose[2])
                act.initpose.append(pos)
            cond_sim.actorlist.append(act)
        return cond_sim

    def world_to_carla(self, x, y, alfa):
        # carla_point = self.R @ (np.array([x/1000, y/1000]) - self.trans).T
        carla_point = self.R @ (self.T @ np.array([x/1000, y/1000, 1.0]).T)
        alfa = -np.rad2deg(alfa)-74
        return [-carla_point[0], carla_point[1], alfa]
    ######################
    # From the RoboCompCarla you can call this methods:
    # self.carla_proxy.getState(...)
    # self.carla_proxy.setSimulationParam(...)

    ######################
    # From the RoboCompCarla you can use this types:
    # RoboCompCarla.Posedata
    # RoboCompCarla.Actor
    # RoboCompCarla.Simulations
    # RoboCompCarla.Collision
    # RoboCompCarla.Simresult
    # RoboCompCarla.Results



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
