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
import time
import sys
from threading import Lock
mutex = Lock()
from rich.console import Console
import interfaces as ifaces
from genericworker import *
from sim_manger import *
from multiprocessing import Process
import multiprocessing
import subprocess
import json
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
import threading
import queue
import psutil


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

def carla_fun():
    ports = [2000, 4000, 10000, 15000, 20000, 40000, 60000, 63000]
    tm_ports = [8000, 12000, 14000, 16000, 22000, 24000, 30000, 35000]
    port_list = list(zip(ports, tm_ports))
    z = 8
    servers = []
    for i in range(0, z):
        p, t = port_list[i]
        print(p, t)
        client = carla.Client("127.0.0.1", p)
        client.set_timeout(10.0)
        client.load_world("Arriba2")
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        settings.no_rendering_mode = True
        world.apply_settings(settings)
        print(world.get_actors())
        # world.get_spectator().set_transform(carla.Transform(carla.Location(25, 90, 376)))
        tm = client.get_trafficmanager(t)
        # tm.set_synchronous_mode(True)
        servers.append({'client': client,
                        'world': world,
                        'tm': tm
                        })

    return servers


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=True, servers=carla_fun()):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        self.time_ini = 0.0
        self.n_simulations = 5
        self.cond_simulation = None

        self.results = {
            'valid': False,
            'n_simulation': 0,
            'fullResult': [],
            'time': 0
        }

        self.revision_time = 3
        self.servers = servers
        # print(self.servers)
        self.fixed_delta_seconds = 0.1

    def __del__(self):
        """Destructor"""
        # for actor in self.world.get_actors():
        #     actor.destroy()
        return True

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        my_queue = queue.Queue()
        if self.cond_simulation is not None:

            def storeInQueue(f):
                def wrapper(*args):
                    my_queue.put(f(*args))
                return wrapper

            @storeInQueue
            def multitask(actor_list,servers, duration, revision_time, fixed_delta, name, n_sim):
                # print(name)
                cpu_usage = []
                compute_time = []
                j = 0
                full_results = []
                while j < n_sim:
                    real_time = 0.0
                    time_ini1 = time.time()
                    s = SimManager(actor_list, servers)
                    #print("time initttt", time.time()-time_ini1)
                    i = 1
                    save_actor = False
                    # k = 0
                    while real_time < duration:
                        time_ini = time.time()
                        # if name == 'sim0' and real_time >= k:
                        #     cpu_usage.append(psutil.cpu_percent())
                        #     k += 1
                        # print(real_time)
                        if real_time > revision_time * i:
                            save_actor = True
                            i += 1
                        # if s.sim_register['collision']['iscollision'] == True:
                        #     s.sim_register['collision']['timecollision'] = real_time
                        s.world_tick(save_actor, real_time)
                        save_actor = False
                        real_time += fixed_delta
                        compute_time.append(time.time()-time_ini)
                    full_results.append(s.get_results())
                    del s
                    j += 1
                # if name == 'sim0':
                #     print("CPU usage", sum(cpu_usage)/len(cpu_usage))
                #     print("Total", cpu_usage)
                return full_results
            # @storeInQueue
            # def multitask(actor_list,servers, duration, revision_time, fixed_delta, name):
            #     # print(name)
            #     cpu_usage = []
            #     compute_time = []
            #     real_time = 0.0
            #     time_ini1 = time.time()
            #     s = SimManager(actor_list, servers)
            #     #print("time initttt", time.time()-time_ini1)
            #     i = 1
            #     save_actor = False
            #     while real_time < duration:
            #         time_ini = time.time()
            #         if name == 'sim0' and real_time >= 1:
            #             cpu_usage.append(psutil.cpu_percent())
            #         # print(real_time)
            #         if real_time > revision_time * i:
            #             save_actor = True
            #             i += 1
            #         # if s.sim_register['collision']['iscollision'] == True:
            #         #     s.sim_register['collision']['timecollision'] = real_time
            #         s.world_tick(save_actor, real_time)
            #         save_actor = False
            #         real_time += fixed_delta
            #         compute_time.append(time.time()-time_ini)
            #     results = s.get_results()
            #     del s
            #     if name == 'sim0':
            #         print("CPU usage", sum(cpu_usage)/len(cpu_usage))
            #         # print("Compute_time", sum(compute_time) / len(compute_time))
            #     return results

            self.time_ini = time.time()
            c = 0
            results = []
            tim1 = time.time()
            for i in self.servers:
                t1 = threading.Thread(name=f'sim{c}', target=multitask, args=(self.cond_simulation['actorList'],i,self.cond_simulation["duration"], self.revision_time, self.fixed_delta_seconds, f'sim{c}', 1))
                t1.start()
                #t1.join()
                print("EMPIEZA HILO", f'sim{c}' , time.time() - tim1)
                # results.append(my_queue.get())
                # if c == len(self.servers):
                #     t1.join()
                c += 1
            for _ in self.servers:
                aux = my_queue.get()
                for i in aux:
                    results.append(i)
                # results.append(my_queue.get())
            # results.append(my_queue.get())
            # for job in jobs:
            #     job.kill()

            # print(results)
            self.store_results(results)
        # else:
            # print("Waiting for conditions...")

    def store_results(self, results):
        self.cond_simulation = None
        self.results = {
            'valid': True,
            'n_simulation ': self.n_simulations,
            'fullResult': results,
            'time': time.time() - self.time_ini
        }

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # CARLA INTERFACE IMPLEMENTATION
    def Carla_getState(self):
        ret = ifaces.RoboCompCarla.Results()
        if self.results['valid']:
            # print("RESULTADO VALIDO")
            # print(self.results['time'])
            ret.time = self.results['time']
            print("Tiempo de simulacion", ret.time)
            ret.nsimulation = self.n_simulations
            ret.valid = self.results['valid']
            ret.fullresult = ifaces.Fullresults()
            for r in self.results['fullResult']:
                result = ifaces.RoboCompCarla.Simresult()
                # collision = ifaces.RoboCompCarla.Collision(r['collision']['iscollision'],r['collision']['iscollision'],r['timecollision']['actorcollision'])
                collision = ifaces.RoboCompCarla.Collision(r['collision']['iscollision'], r['collision']['timecollision'], r['collision']['actorcollision'])
                result.collision = collision
                result.isbrake = r['brake']
                result.actorlist = ifaces.Actors()
                for actor in r['actorList']:
                    act = ifaces.RoboCompCarla.Actor(actor['id'], actor['carlaID'], ifaces.Fullposedata(), ifaces.Fullposedata(), actor['rol'])
                    for pose in actor['fullPose']:
                        act.pose.append(ifaces.RoboCompCarla.Posedata(pose[0], pose[1], 0, 0, 0, pose[2]))
                    result.actorlist.append(act)
                ret.fullresult.append(result)
            self.results['valid'] = False
        return ret

    def Carla_setSimulationParam(self, condini):
        if not self.cond_simulation and condini.duration > 8:
            actor_list = []
            for actor in condini.actorlist:
                act = {
                    'id': actor.id,
                    'carlaID': actor.carlaid,
                    'initPose': [],
                    'fullPose': [],
                    'rol': actor.rol
                }
                for pose in actor.initpose:
                    act['initPose'].append([pose.tx, pose.ty, pose.rz])
                actor_list.append(act)
            self.cond_simulation = {
                "n_simulation": condini.nsimulation,
                "duration": condini.duration,
                "actorList": actor_list
            }
            # print("New simulations arrive:", self.cond_simulation["n_simulation"])
        # else:
            # print("Simulating....")

