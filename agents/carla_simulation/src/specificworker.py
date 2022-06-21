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
# from sim_manager import *
from sim_manger import *


sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

def carla_fun():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    client.load_world("Arriba2")
    world = client.get_world()
    world.get_spectator().set_transform(carla.Transform(carla.Location(25, 90, 376)))
    tm = client.get_trafficmanager(8000)

    return [client, world, tm]


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=True):
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

        self.revision_time = 5

        self.sim_params = [
            {  # TODO: crear una por cada simulación
                'port': 2000,
                'tm_port': 8000,
                'actor_list': [],
            },

            {  # TODO: crear una por cada simulación
                'port': 20000,
                'tm_port': 10000,
                'actor_list': [],
            }
        ]

        self.fixed_delta_seconds = 0.1


    def __del__(self):
        """Destructor"""
        for actor in self.world.get_actors():
            actor.destroy

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        '''
        compute:
            Esperar por las condiciones de las simulaciones nuevas
            while(simular)
                tick de todas las simulaciones
            resultados de las simulaciones
            pasar a la interfaz
        '''

        if self.cond_simulation is not None:

            # Creación de las simulaciones nuevas
            self.time_ini = time.time()
            duration = self.cond_simulation["duration"]
            simulations = []
            real_time = 0
            for i in range(self.cond_simulation["n_simulations"]):
                simulations.append(SimManager(self.sim_params[i]))

            # Simular
            while real_time < duration:
                for s in simulations:
                    s.world_tick()
                real_time += self.fixed_delta_seconds

            #Obtener resultados
            results = [s.get_results for s in simulations]
            self.store_results(results)

        else:
            print("Waiting for conditions...")

    def store_results(self, results):
        self.cond_simulation = None
        self.results['fullResult'] = results
        self.results['time'] = time.time() - self.time_ini
        self.results['valid'] = True
        self.results['n_simulation'] = self.n_simulations
        print(self.results)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # CARLA INTERFACE IMPLEMENTATION
    def Carla_getState(self):
        ret = ifaces.RoboCompCarla.Results()
        if self.results['valid']:
            print("RESULTADO VALIDO")
            ret.time = time.time() - self.time_ini
            ret.nsimulation = self.n_simulations
            ret.valid = self.results['valid']
            ret.fullresult = ifaces.Fullresults()
            for r in self.results['fullResult']:
                result = ifaces.RoboCompCarla.Simresult()
                result.colision = r['collision']
                result.isbrake = r['isBrake']
                result.actorlist = ifaces.Actors()
                for actor in r['actorList']:
                    act = ifaces.RoboCompCarla.Actor(actor['id'], actor['carlaID'], ifaces.Fullposedata(), actor['rol'])
                    for pose in actor['fullPose']:
                        act.pose.append(ifaces.RoboCompCarla.Posedata(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    result.actorlist.append(act)
                ret.fullresult.append(result)
            self.results['valid'] = False
        return ret

    def Carla_setSimulationParam(self, condini):
        if not self.cond_simulation:
            actor_list = []
            for actor in condini.actorlist:
                act = {
                    'id': actor.id,
                    'carlaID': actor.carlaid,
                    'fullPose': [],
                    'rol': actor.rol
                }
                for pose in actor.pose:
                    act['fullPose'].append([pose.tx, pose.ty, pose.tz, pose.rx, pose.ry, pose.rz])
                actor_list.append(act)
            self.cond_simulation = {
                "n_simulation": condini.nsimulation,
                "duration": condini.duration,
                "actorList": actor_list
            }
            print("New simulations arrive:", self.cond_simulation)
        else:
            print("Simulating....")

