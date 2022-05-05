import pygame
import carla
import cv2
import numpy as np
import time
import random

from threading import Lock

mutex = Lock()
from rich.console import Console

console = Console(highlight=False)


class VehicleType:
    def __init__(self, id=None, pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]):
        self.id = id
        self.pos = pos
        self.rot = rot


class Simulation:
    def __init__(self, map='Town01'):
        self.client = None
        self.world = None
        self._blueprint_library = None
        self.ego_vehicle = None
        self.vehicles = []
        self.INPUT_WIDTH = 424
        self.INPUT_HEIGHT = 240
        self.car_rgb_cams = {}
        self._server_clock = pygame.time.Clock()
        self.create_client() #Inicializamos conexión con Carla
        self.client.set_timeout(2.0)
        self.initialize_world(map) #Cargamos Mapa (En este caso Town01, hay que cambiar por el nuestro)
        # self.world.wait_for_tick()

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
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            self.world.apply_settings(settings)
            print('Done')
            # settings = self.world.get_settings()
            # settings.synchronous_mode = True
            # self.world.apply_settings(settings)
            print(f'Loading world took {time.time() - init_time:2.2f} seconds')
            self._blueprint_library = self.world.get_blueprint_library()
        else:
            console.log("No carla client created. Call create_client first.", style="red")

    def load_vehicles(self, vehicle_list):
        ego, *vehicles = vehicle_list
        ego_bp = self._blueprint_library.find('vehicle.tesla.model3') #Nuestro vehiculo
        ego_bp.set_attribute('role_name', 'ego')
        try:
            spawn_point = carla.Transform(carla.Location(x=ego.pos[0], y=ego.pos[1], z=ego.pos[2] + 2),
                                          carla.Rotation(ego.rot[0], ego.rot[1],
                                                         ego.rot[2]))  # Comprobar angulos CARLA-ROBOCOMP
        except:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            print("Error to loaded vehicle")
        self.vehicles.append(self.world.try_spawn_actor(ego_bp, spawn_point))
        self.set_ego_rgb_frontal_cams(self.vehicles[0], 3)
        for vehicle in vehicles:
            choices = self.world.get_blueprint_library().filter('vehicle.*')
            vehicle_blueprint = random.choice(choices)
            try:
                spawn_point = carla.Transform(carla.Location(x=vehicle.pos[0], y=vehicle.pos[1], z=vehicle.pos[2] + 2), carla.Rotation(vehicle.rot[0], vehicle.rot[1], vehicle.rot[2])) #Comprobar angulos CARLA-ROBOCOMP
                self.vehicles.append(self.world.try_spawn_actor(vehicle_blueprint, spawn_point))
            except:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                print("Error to loaded vehicle")
            self.vehicles.append(self.world.try_spawn_actor(vehicle_blueprint, spawn_point))

    def set_ego_rgb_frontal_cams(self, ego_vehicle, num_cams):
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
                self.center_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.ego_vehicle,
                                                         attachment_type=carla.AttachmentType.Rigid)
                self.center_cam.listen(lambda image: self.sensor_rgb_callback(image, "CenterCam"))
            elif i == 1:
                left_cam_location = carla.Location(dimensiones_car.x, -0.02, dimensiones_car.z + 1)
                left_cam_rotation = carla.Rotation(0, -60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                left_cam_transform = carla.Transform(left_cam_location, left_cam_rotation)
                self.left_cam = self.world.spawn_actor(cam_bp, left_cam_transform, attach_to=self.ego_vehicle,
                                                       attachment_type=carla.AttachmentType.Rigid)
                self.left_cam.listen(lambda image: self.sensor_rgb_callback(image, "LeftCam"))
            elif i == 2:
                right_cam_location = carla.Location(dimensiones_car.x, 0.02, dimensiones_car.z + 1)
                right_cam_rotation = carla.Rotation(0, 60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                right_cam_transform = carla.Transform(right_cam_location, right_cam_rotation)

                self.right_cam = self.world.spawn_actor(cam_bp, right_cam_transform, attach_to=self.ego_vehicle,
                                                        attachment_type=carla.AttachmentType.Rigid)

                self.right_cam.listen(lambda image: self.sensor_rgb_callback(image, "RightCam"))

    # Compute np array for rgb sensors
    def sensor_rgb_callback(self, img, sensorID):
        global mutex
        mutex.acquire()
        array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (self.INPUT_WIDTH, self.INPUT_HEIGHT, 4))
        array = array[:, :, :3]
        self.car_rgb_cams[sensorID] = array
        mutex.release()

    def mosaic(self):
        v_f = cv2.hconcat([self.car_rgb_cams["LeftCam"], self.car_rgb_cams["CenterCam"]])
        v_f = cv2.hconcat([v_f, self.car_rgb_cams["RightCam"]])
        cv2.imshow("FrontCam", v_f)
        cv2.waitKey(1)

    # Dudas de que hace
    # def on_world_tick(self):
    #     self._server_clock.tick()
    #     self.server_fps = self._server_clock.get_fps()
    #     if self.server_fps in [float("-inf"), float("inf")]:
    #         self.server_fps = -1
    #     print('Server FPS', int(self.server_fps))

    def __del__(self):
        if self.world is not None:
            for actor in self.world.get_actors():
                actor.destroy()
            self.world.destroy()
        print('Simulation destruction')