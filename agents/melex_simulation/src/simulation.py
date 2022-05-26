# import pygame
import carla
import cv2
import numpy as np
import time
import random
import csv

from threading import Lock

mutex = Lock()
from rich.console import Console

console = Console(highlight=False)


class ActorType:
    def __init__(self, node_id=None, carla_id=None, rol="actor", pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]):
        self.node_id = node_id
        self.carla_id = carla_id
        self.rol = rol
        self.pos = pos
        self.rot = rot


class Simulation:
    def __init__(self, map='Town01'):
        self.client = None
        self.world = None
        self._blueprint_library = None
        self.carla_actors = []
        self.collisions = []
        # self.spawn_points = []
        self.INPUT_WIDTH = 424
        self.INPUT_HEIGHT = 240
        self.car_rgb_cams = {}
        # self._server_clock = pygame.time.Clock()
        self.create_client() #Inicializamos conexión con Carla
        # pygame.init()
        self.initialize_world(map) #Cargamos Mapa (En este caso Town01, hay que cambiar por el nuestro)
        # text_file = open("./etc/CampusV16.xodr", "r")
        # self.map = carla.Map("CampusV16", text_file.read())
        # self.load_spawn_points()
        print(self.world)
        self.spawn_points = self.world.get_map().get_spawn_points()
        print("ANTES")
        self.tm = self.client.get_trafficmanager(8000)
        print("DESPUES")
        # self.tm = self.client.get_trafficmanager()
        # self.tm.set_synchronous_mode(True)
        # self.tm_port = self.tm.get_port()
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
            self.init_time = time.time()
            print('Loading world...')
            self.client.load_world(map_name)
            self.world = self.client.get_world()
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)
            print(f'Loading world took {time.time() - self.init_time:2.2f} seconds')
            self._blueprint_library = self.world.get_blueprint_library()
        else:
            console.log("No carla client created. Call create_client first.", style="red")

    def load_spawn_points(self):
        file = open('etc/spawn_points.csv')
        csvreader = csv.reader(file)
        for row in csvreader:
            self.spawn_points.append(carla.Transform(carla.Location(float(row[0]), float(row[1]), float(row[2])), carla.Rotation(float(row[3]), float(row[4]), float(row[5]))))
        file.close()

    def load_actors(self, actors):
        for actor in actors:
            print(actor.rol)
            if actor.rol == "ego_vehicle":
                bp = self._blueprint_library.find('vehicle.tesla.model3') #Nuestro vehiculo
                bp.set_attribute('role_name', 'ego')
            elif actor.rol == "vehicle":
                bp = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
            elif actor.rol == "people":
                bp = random.choice(self.world.get_blueprint_library().filter('pederstian.*'))
            else:
                print("Unknown actor")
            try:
                self.spawn_point = carla.Transform(carla.Location(x=actor.pos[0], y=actor.pos[1], z=actor.pos[2] + 385),
                                              carla.Rotation(actor.rot[0], actor.rot[1], actor.rot[2]))  # Comprobar angulos CARLA-ROBOCOMP
            except:
                self.spawn_point = random.choice(self.spawn_points) if self.spawn_points else carla.Transform()
                # spawn_point = self.world.get_random_location_from_navigation()
                # print("Error to loaded vehicle")
            # self.world.spawn_actor(bp, spawn_point)
            self.carla_actors.append(self.world.spawn_actor(bp, self.spawn_point))
            print(self.carla_actors)
            actor.carla_id = self.carla_actors[-1].id
        self.set_ego_sensors(self.carla_actors[0], 3)


    def set_ego_sensors(self, ego_vehicle, num_cams):
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
                self.center_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
                                                         attachment_type=carla.AttachmentType.Rigid)
                self.center_cam.listen(lambda image: self.sensor_rgb_callback(image, "CenterCam"))
            elif i == 1:
                left_cam_location = carla.Location(dimensiones_car.x, -0.02, dimensiones_car.z + 1)
                left_cam_rotation = carla.Rotation(0, -60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                left_cam_transform = carla.Transform(left_cam_location, left_cam_rotation)
                self.left_cam = self.world.spawn_actor(cam_bp, left_cam_transform, attach_to=ego_vehicle,
                                                       attachment_type=carla.AttachmentType.Rigid)
                self.left_cam.listen(lambda image: self.sensor_rgb_callback(image, "LeftCam"))
            elif i == 2:
                right_cam_location = carla.Location(dimensiones_car.x, 0.02, dimensiones_car.z + 1)
                right_cam_rotation = carla.Rotation(0, 60, 0)
                # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
                right_cam_transform = carla.Transform(right_cam_location, right_cam_rotation)

                self.right_cam = self.world.spawn_actor(cam_bp, right_cam_transform, attach_to=ego_vehicle,
                                                        attachment_type=carla.AttachmentType.Rigid)

                self.right_cam.listen(lambda image: self.sensor_rgb_callback(image, "RightCam"))
        collision_bp = self._blueprint_library.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(collision_bp, cam_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.collision_sensor.listen(lambda collision: self.collision_callback(collision))

    # Compute np array for rgb sensors
    def sensor_rgb_callback(self, img, sensor_id):
        global mutex
        mutex.acquire()
        array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (self.INPUT_WIDTH, self.INPUT_HEIGHT, 4))
        array = array[:, :, :3]
        self.car_rgb_cams[sensor_id] = array
        mutex.release()

    def mosaic(self):
        v_f = cv2.hconcat([self.car_rgb_cams["LeftCam"], self.car_rgb_cams["CenterCam"]])
        v_f = cv2.hconcat([v_f, self.car_rgb_cams["RightCam"]])
        cv2.imshow("FrontCam", v_f)
        cv2.waitKey(1)

    def collision_callback(self, collision):
        print("Collision with", collision.other_actor.type_id)
        self.collisions.append(collision.other_actor)
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