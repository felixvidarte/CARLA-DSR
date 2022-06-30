import carla
import random
import numpy as np
import scipy.stats as scp
import copy
import cv2
import time
import weakref
from threading import Lock
mutex = Lock()

# def carla_fun():
#     ports = [2000, 20000, 40000, 60000, 63000]
#     tm_ports = [8000, 16000, 24000, 30000, 35000]
#     servers = []
#     for p, t in list(zip(ports, tm_ports)):
#         print(p, t)
#         client = carla.Client("127.0.0.1", p)
#         client.set_timeout(10.0)
#         client.load_world("Arriba2")
#         world = client.get_world()
#         # world.get_spectator().set_transform(carla.Transform(carla.Location(25, 90, 376)))
#         tm = client.get_trafficmanager(t)
#         servers.append({'client': client,
#                         'world': world,
#                         'tm': tm
#                         })
#
#     return servers


class SimManager:
    def __init__(self, actor_list, servers):
        time1 = time.time()
        self.client = servers['client']
        self.loop = None
        self.world = servers['world']
        self.tm = servers['tm']
        # settings = self.world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.1
        # settings.no_rendering_mode = True
        # self.world.apply_settings(settings)

        self._blueprint_library = self.world.get_blueprint_library()

        self.car_rgb_cams = {}
        self.collision_sensor = None

        self.sim_register = {
            'brake': False,
            'collision': {
                'iscollision': False,
                'timecollision': 0,
                'actorcollision': 0
            },
            'actorList': copy.deepcopy(actor_list),
        }

        # self.tm.set_synchronous_mode(True)
        # self.tm.global_percentage_speed_difference(0)
        self.real_time = 0
        self.max_velocity = 7
        self.revision_time = 8.5
        # self.destroy_actor()
        self.world.tick()
        self.__load_actors()

    def __del__(self):
        self.destroy_actor()

    def world_tick(self, save_actors, real_time):#, show):
        #print(self.world.get_actors())
        #print(self.car_rgb_cams)
        # self.mosaic()
        self.real_time = real_time
        self.world.tick()
        # if show:
        #     self.mosaic()
        ego_id = self.sim_register["actorList"][0]["carlaID"]
        self.sim_register["brake"] |= self.world.get_actor(ego_id).get_control().brake > 0.6
        if save_actors:
            self.__add_actor_pose()

    def get_results(self):
        resultados = copy.deepcopy(self.sim_register)
        #print(resultados)
        return resultados


    '''PRIVATE AUX METHODS'''
    def __load_actors(self):
        for actor in self.sim_register["actorList"]:
            # print(actor['rol'])
            if actor['rol'] == 'ego_vehicle':
                bp = self._blueprint_library.find('vehicle.tesla.model3')  # Nuestro vehiculo
                bp.set_attribute('role_name', 'ego')
            elif actor['rol'] == 'vehicle':
                bp = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
            elif actor['rol'] == 'person':
                bp = random.choice(self.world.get_blueprint_library().filter('walker.*'))
            else:
                print("Unknown actor")
            spawn_point = carla.Transform(carla.Location(x=actor['initPose'][-1][0], y=actor['initPose'][-1][1], z=373), carla.Rotation(0, actor['initPose'][-1][2], 0))  # Comprobar angulos CARLA-ROBOCOMP
            # print(spawn_point)
            # carla_actors = []
            new_actor = self.world.spawn_actor(bp, spawn_point)
            if bp.get_attribute("role_name") == "ego":
                self.__set_ego_sensors(new_actor, 3)
            self.__apply_control(new_actor, actor)
            actor['carlaID'] = new_actor.id
                # carla_actors.append(new_actor)
                # print(self.carla_actors)
            # except Exception as e:
            #     print(e)

    def destroy_actor(self):
        if self.collision_sensor is not None:
            self.collision_sensor.destroy()
            self.collision_sensor = None
        for actor in self.world.get_actors():
            if 'sensor' in actor.type_id:
                pass
            else:
                actor.destroy()
            # print(actor_destroy)

    def __apply_control(self, actor, actor_dict):
        if actor_dict['rol'] == 'ego_vehicle':
            actor.apply_control(carla.VehicleControl(0.2))
            actor.set_autopilot(True, self.tm.get_port())
            self.tm.keep_right_rule_percentage(actor, 100.0)
            self.tm.ignore_walkers_percentage(actor, 100.0)
            self.tm.ignore_vehicles_percentage(actor, 100.0)
            self.tm.vehicle_percentage_speed_difference(actor, self.__vehicle_control(actor_dict['initPose']))
            # self.tm.set_desired_speed(actor, 10)
        elif actor_dict['rol'] == 'vehicle':
            actor.set_autopilot(True, self.tm.get_port())
            self.tm.ignore_signs_percentage(actor, random.random() * 100)
            self.tm.vehicle_percentage_speed_difference(actor, self.__vehicle_control(actor_dict['initPose']))
            self.tm.ignore_vehicles_percentage(actor, 100)
        elif actor_dict['rol'] == 'person':
            actor.apply_control(self.__person_control(actor_dict['initPose']))

    def __person_control(self, person_pose):
        pos = np.array(person_pose[-1])
        direction = pos[2]
        speed = 1.5
        desv_direct = 30
        if len(person_pose) > 1:
            pos_ant = np.array(person_pose[-2])
            velocity = (pos_ant - pos) / self.revision_time
            current_speed = np.linalg.norm(velocity)
            if current_speed > 0.1:
                direction = np.rad2deg(np.arctan2(velocity[1], velocity[0]))
                speed = current_speed
                desv_direct = 10
            else:
                speed *= 0.4
        direction = self.__gaussian_distribution(direction, desv_direct)
        speed = np.abs(self.__gaussian_distribution(speed, speed/3))
        [x, y] = [np.cos(np.rad2deg(direction)), np.sin(np.rad2deg(direction))]
        vector_direction = carla.Vector3D(x=x, y=y, z=0.0)
        # print("People control", vector_direction, speed)
        return carla.WalkerControl(vector_direction, speed)

    def __vehicle_control(self, vehicle_pose):
        pos = np.array(vehicle_pose[-1])
        current_velocity = self.max_velocity
        if len(vehicle_pose) > 1:
            pos_ant = np.array(vehicle_pose[-2])
            velocity = np.linalg.norm(pos_ant-pos) / self.revision_time
            if velocity > 0.1:
                current_velocity = velocity
            else:
                current_velocity = current_velocity / 2
        porcentage = (self.max_velocity - current_velocity)/self.max_velocity*100
        porcentage = self.__gaussian_distribution(porcentage, 10)
        return porcentage

    def __gaussian_distribution(self, mean, std):
        dist_gauss = scp.norm(mean, std)
        valor = dist_gauss.rvs()
        return valor

    def __set_ego_sensors(self, ego_vehicle, num_cams):
        #print('SET_SENSORS')
        weak_self = weakref.ref(self)
        # self.INPUT_WIDTH = 424
        # self.INPUT_HEIGHT = 240
        # cam_bp = self._blueprint_library.find('sensor.camera.rgb')
        # cam_bp.set_attribute("image_size_x", str(self.INPUT_HEIGHT))
        # cam_bp.set_attribute("image_size_y", str(self.INPUT_WIDTH))
        # cam_bp.set_attribute("fov", str(65))
        dimensiones_car = ego_vehicle.bounding_box.extent
        colision_sensor_location = carla.Location(dimensiones_car.x + 0.02, 0, dimensiones_car.z + 1)
        # In carla location is (pitch, yaw, roll) where pitch is y, yaw is z and roll is x

        # for i in range(num_cams):
        #     if i == 0:
        #         center_cam_location = carla.Location(dimensiones_car.x + 0.02, 0, dimensiones_car.z + 1)
        #         center_cam_rotation = carla.Rotation(0, 0, 0)
        #         cam_transform = carla.Transform(center_cam_location, center_cam_rotation)
        #         self.center_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
        #                                                  attachment_type=carla.AttachmentType.Rigid)
        #         self.center_cam.listen(lambda image: self.sensor_rgb_callback(image, "CenterCam"))
        #     elif i == 1:
        #         left_cam_location = carla.Location(dimensiones_car.x, -0.02, dimensiones_car.z + 1)
        #         left_cam_rotation = carla.Rotation(0, -60, 0)
        #         # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
        #         left_cam_transform = carla.Transform(left_cam_location, left_cam_rotation)
        #         self.left_cam = self.world.spawn_actor(cam_bp, left_cam_transform, attach_to=ego_vehicle,
        #                                                attachment_type=carla.AttachmentType.Rigid)
        #         self.left_cam.listen(lambda image: self.sensor_rgb_callback(image, "LeftCam"))
        #     elif i == 2:
        #         right_cam_location = carla.Location(dimensiones_car.x, 0.02, dimensiones_car.z + 1)
        #         right_cam_rotation = carla.Rotation(0, 60, 0)
        #         # left_cam_rotation = carla.Rotation(0, 180+math.degrees(-0.244346), math.degrees(-np.pi/3))
        #         right_cam_transform = carla.Transform(right_cam_location, right_cam_rotation)
        #
        #         self.right_cam = self.world.spawn_actor(cam_bp, right_cam_transform, attach_to=ego_vehicle,
        #                                                 attachment_type=carla.AttachmentType.Rigid)
        #
        #         self.right_cam.listen(lambda image: self.sensor_rgb_callback(image, "RightCam"))
        collision_bp = self._blueprint_library.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(location=colision_sensor_location, rotation=carla.Rotation()), attach_to=ego_vehicle)
        self.collision_sensor.listen(lambda collision: SimManager.__collision_callback(weak_self, collision))

    def __add_actor_pose(self): # TODO
        for actor in self.sim_register["actorList"]:
            actor_pose = self.world.get_actor(actor['carlaID']).get_transform()
            pose = [actor_pose.location.x, actor_pose.location.y, actor_pose.rotation.yaw]
            actor['fullPose'].append(pose)


    @staticmethod
    def __collision_callback(weak_self, collision):  # TODO: review
        self = weak_self()
        if not self:
            return
        if 'static' not in collision.other_actor.type_id and not self.sim_register['collision']['iscollision']:
            #print("Collision with", collision.other_actor.type_id, collision.other_actor.id)
            self.sim_register["collision"] = {
                'iscollision': True,
                'timecollision': self.real_time,
                'actorcollision': 0
            }
            for actor in self.sim_register['actorList']:
                if actor['carlaID'] == collision.other_actor.id:
                    self.sim_register['collision']['actorcollision'] = actor['id']
    #
    # def sensor_rgb_callback(self, img, sensor_id):
    #     print('PASA')
    #     global mutex
    #     mutex.acquire()
    #     array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
    #     array = np.reshape(array, (self.INPUT_WIDTH, self.INPUT_HEIGHT, 4))
    #     array = array[:, :, :3]
    #     self.car_rgb_cams[sensor_id] = array
    #     print('FINCALLBACK')
    #     mutex.release()
    #
    # def mosaic(self):
    #     if len(self.car_rgb_cams.keys()) == 3:
    #         v_f = cv2.hconcat([self.car_rgb_cams["LeftCam"], self.car_rgb_cams["CenterCam"]])
    #         v_f = cv2.hconcat([v_f, self.car_rgb_cams["RightCam"]])
    #         cv2.imshow("FrontCam", v_f)
    #         cv2.waitKey(1)
    #     else:
    #         print("Wait for images")
