import carla
import random
import numpy as np
import scipy.stats as scp


class SimManager:
    def __init__(self, data):
        self.client = carla.client('localhost', data['port'])
        self.client.load_world("Arriba2")
        self.world = self.client.get_world()
        self.world.get_spectator().set_transform(carla.Transform(carla.Location(25, 90, 376)))
        self.tm = self.client.get_trafficmanager(data['tm_port'])
        self.client.set_timeout(10.0)
        print('Loading world...')
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        settings.no_rendering_mode = True
        # settings.substepping = True
        # settings.max_substep_delta_time = 0.01
        # settings.max_substeps = 10
        self.world.apply_settings(settings)
        self.spawn_points = self.world.get_map().get_spawn_points()
        self._blueprint_library = self.world.get_blueprint_library()

        self.sim_register = {
            'brake': False,
            'collision': False,
            'actor_list': data["actor_list"],
        }

        self.tm.set_synchronous_mode(True)
        self.tm.global_percentage_speed_difference(50)
        self.max_velocity = 7

        self.__load_actors()

    def __del__(self):
        """Destructor"""
        for actor in self.world.get_actors():
            actor.destroy()

    def world_tick(self, save_actors):
        self.world.tick()
        ego_id = self.sim_register["actor_list"][0]["carlaId"]
        self.sim_register["brake"] |= self.world.get_actor(ego_id).get_control().brake > 0.5
        if save_actors:
            self.__add_actor_pose()
        self.real_time += self.world.get_settings().fixed_delta_seconds

    def get_results(self):
        return self.sim_register

    '''PRIVATE AUX METHODS'''
    def __load_actors(self):
        for actor in self.sim_register["actor_list"]:
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

            spawn_point = carla.Transform(carla.Location(x=actor['fullPose'][-1][0], y=actor['fullPose'][-1][1], z=370), carla.Rotation(actor['fullPose'][-1][4], actor['fullPose'][-1][5], actor['fullPose'][-1][3]))  # Comprobar angulos CARLA-ROBOCOMP
            # print(spawn_point)
            carla_actors = []
            try:
                new_actor = self.world.spawn_actor(bp, spawn_point)
                if bp.get_attribute(new_actor("role_name")) == "ego_vehicle":
                    self.__set_ego_sensors(new_actor, 3)
                self.__apply_control(new_actor, actor)
                actor['carlaID'] = new_actor.id
                carla_actors.append(new_actor)
                # print(self.carla_actors)
            except Exception as e:
                print(e)

    def __apply_control(self, actor, actor_dict):
        if actor_dict['rol'] == 'ego_vehicle':
            actor.set_autopilot(True)
            # self.tm.keep_right_rule_percentage(actor, 100.0)
            self.tm.ignore_walkers_percentage(actor, 100.0)
            self.tm.ignore_vehicles_percentage(actor, 100.0)
            # self.tm.set_desired_speed(actor, 10)
        elif actor_dict['rol'] == 'vehicle':
            actor.set_autopilot(True)
            self.tm.ignore_signs_percentage(actor, random.random() * 100)
            self.tm.vehicle_percentage_speed_difference(actor, random.random() * 100)
            self.tm.ignore_vehicles_percentage(actor, 100)
        elif actor_dict['rol'] == 'person':
            print("PEOPLE APPLY CONTROL")
            actor.apply_control(self.__person_control(actor_dict['fullPose']))

    def __person_control(self, person_pose):
        pos = np.array(person_pose[-1])
        direction = pos[5]
        speed = 1.5
        desv_direct = 20
        if len(person_pose) > 1:
            pos_ant = np.array(person_pose[-2])
            velocity = pos_ant - pos
            current_speed = np.linalg.norm(velocity)/5
            if current_speed > 0.1:
                direction = np.rad2deg(np.arctan2(velocity[1], velocity[0]))
                speed = np.linalg.norm(velocity) / 5
                desv_direct = 10
        dist_gauss_direction = scp.norm(direction, desv_direct)
        dist_gauss_speed = scp.norm(speed, speed/3)
        direction = dist_gauss_direction.rvs()
        [x, y] = [np.cos(np.rad2deg(direction)), np.sin(np.rad2deg(direction))]
        vector_direction = carla.Vector3D(x=x, y=y, z=0.0)
        speed = dist_gauss_speed.rvs()
        print(vector_direction,speed)
        return carla.WalkerControl(vector_direction, speed)

    def __vehicle_control(self, vehicle_pose):
        pos = np.array(vehicle_pose[-1])
        current_velocity = self.max_velocity
        if len(vehicle_pose) > 1:
            pos_ant = np.array(vehicle_pose[-2])
            velocity = np.linalg.norm(pos_ant-pos) / 5
            if velocity > 0.1:
                current_velocity = velocity
        porcentage = (self.max_velocity - current_velocity)/self.max_velocity*100
        porcentage = scp.norm(porcentage, np.abs(porcentage/3))
        return porcentage

    def __set_ego_sensors(self, ego_vehicle, num_cams):
        # self.INPUT_WIDTH = 424
        # self.INPUT_HEIGHT = 240
        # cam_bp = self._blueprint_library.find('sensor.camera.rgb')
        # cam_bp.set_attribute("image_size_x", str(self.INPUT_HEIGHT))
        # cam_bp.set_attribute("image_size_y", str(self.INPUT_WIDTH))
        # cam_bp.set_attribute("fov", str(65))
        dimensiones_car = ego_vehicle.bounding_box.extent
        colision_sensor_location = carla.Location(dimensiones_car.x + 0.02, 0, dimensiones_car.z + 1)
        # # In carla location is (pitch, yaw, roll) where pitch is y, yaw is z and roll is x
        #
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
        collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(location=colision_sensor_location, rotation=carla.Rotation()), attach_to=ego_vehicle,
                                                  attachment_type=carla.AttachmentType.Rigid)
        collision_sensor.listen(lambda collision: self.__collision_callback(collision))

    def __add_actor_pose(self): # TODO
        for actor in self.sim_register["actor_list"]:
            actor_pose = self.world.get_actor(actor['carlaID']).get_transform()
            pose = [actor_pose.location.x, actor_pose.location.y, actor_pose.location.z, actor_pose.rotation.roll, actor_pose.rotation.pitch, actor_pose.rotation.yaw]
            actor['pose_list'].append(pose)

    def __collision_callback(self, collision):  # TODO: review
        # print("Collision with", collision.other_actor.type_id)
        self.sim_register["collisions"] = True
