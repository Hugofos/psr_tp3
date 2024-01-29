#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse
from random import randrange

positions_person = {
    'bedroom': [[-7.527694, 1.922236, 0, 0, 0, 0],
                [-3.926185, 2.417522, 0, 0, 0, 1.605342],
                [-7.767709, -4.543357, 0, 0, 0, 1.605393]],
    'living_room': [[0.392324, -1.621330, 0, 0, 0, 2.753127],
                    [2.645318, 3.883647, 0, 0, 0, 0.581918],
                    [-1.275711, -4.797814, 0, 0, 0, 2.759136]],
    'kitchen': [[7.708288, -3.020768, 0, 0, 0, 1.496065],
                [5.931831, -5.098905, 0, 0, 0, 3.105079],
                [7.990297, 0.975803, 0, 0, 0, -1.710839]]
}

positions_spheres = {
    'bedroom': [[-6.093300, 1.820318, 0.647514, 0, 0, 0],
                [-4.482804, 2.846300, 0.688662, 0, 0, 0],
                [-8.928449, 2.005820, 0.753452, 0, 0, 0]],
    'living_room': [[-0.554201, 4.026761, 0.403854, 0, 0, 0],
                    [1.963260, -1.710975, 0.348381, 0, 0, 0],
                    [1.579539, -5.173858, 0.488614, 0, 0, 0]],
    'kitchen': [[8.953339, -2.430203, 0.906312, 0, 0, 0],
                [7.015892, -5.042284, 0.906312, 0, 0, 0],
                [8.251926, -1.893124, -0.012190, 0, 0, 0]]
}

positions_cubes = {
    'bedroom': [[-7.737369, 2.782921, 0.692083, 0, 0, 0],
                [-4.615532, 2.230153, 0.007420, 0, 0, 0],
                [-8.914158, 0.726259, 0.007420, 0, 0, 0]],
    'living_room': [[3.647012, 3.125789, 0.397628, 0, 0, 0],
                    [-0.231497, -1.336863, 0.496713, 0, 0, 0],
                    [4.378500, -5.195240, 0.689666, 0, 0, 0]],
    'kitchen': [[6.179825, 0.675105, 0.011945, 0, 0, 0],
                [8.878416, -0.274337, 0.011945, 0, 0, 0],
                [8.362491, -4.556175, 0.011945, 0, 0, 0]]
}


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Script to spawn objects in the house')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='house')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='person_standing')
    parser.add_argument('-q', '--quantity', type=int, help='', required=False,
                        default=1)

    args = vars(parser.parse_args())  # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('psr_tp3') + '/robutler_description/models/'

    # Defines poses where to put objects
    poses = {}

    if args['object'] == 'sphere_v':
        for k in positions_spheres:
            for c in positions_spheres[k]:
                p = Pose()
                p.position = Point(x=c[0],y=c[1],z=c[2])
                q = quaternion_from_euler(c[3],c[4],c[5])
                p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                if k in poses.keys():
                    poses[k].append(p)
                else:
                    poses[k] = [p]
                if 'house' in poses.keys():
                    poses['house'].append(p)
                else:
                    poses['house'] = [p]
    elif args['object'] == 'person_standing':
        for k in positions_person:
            for c in positions_person[k]:
                p = Pose()
                p.position = Point(x=c[0],y=c[1],z=c[2])
                q = quaternion_from_euler(c[3],c[4],c[5])
                p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                if k in poses.keys():
                    poses[k].append(p)
                else:
                    poses[k] = [p]
                if 'house' in poses.keys():
                    poses['house'].append(p)
                else:
                    poses['house'] = [p]
    else:
        for k in positions_cubes:
            for c in positions_cubes[k]:
                p = Pose()
                p.position = Point(x=c[0],y=c[1],z=c[2])
                q = quaternion_from_euler(c[3],c[4],c[5])
                p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                if k in poses.keys():
                    poses[k].append(p)
                else:
                    poses[k] = [p]
                if 'house' in poses.keys():
                    poses['house'].append(p)
                else:
                    poses['house'] = [p]

    # on bed pose
    # p = Pose()
    # p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    # q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    # p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    # poses['on_bed'] = {'pose': p}

    # # on bed-side-table pose
    # p = Pose()
    # p.position = Point(x=-4.489786, y=2.867268, z=0.679033)
    # q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    # p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    # poses['on_bed_side_table'] = {'pose': p}

    # define objects
    objects = {}

    # add object sphere_v
    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()}

    # add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person_standing'] = {'name': 'person_standing', 'sdf': f.read()}

    # add object cube_b
    f = open(package_path + 'cube_b/model.sdf', 'r')
    objects['cube_b'] = {'name': 'cube_b', 'sdf': f.read()}
    print(objects)

    # Check if given object and location are valid

    if not args['location'] in poses.keys() :
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))

    if not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))

    # -------------------------------
    # ROS
    # -------------------------------

    rospy.init_node('insert_object', log_level=rospy.INFO)

    service_name = 'gazebo/spawn_sdf_model'
    print('waiting for service ' + service_name + ' ... ', end='')
    rospy.wait_for_service(service_name)
    print('Found')

    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    if args['quantity'] > 3:
        print('Quantity of objects should be equal or lower than 3')
    else:

        print('Spawning object or objects ...')

        
        for i in range(args['quantity']):
            uuid_str = str(uuid.uuid4())
            r = randrange(len(poses[args['location']]))

            service_client(objects[args['object']]['name'] + '_' + uuid_str,
                        objects[args['object']]['sdf'],
                        objects[args['object']]['name'] + '_' + uuid_str,
                        poses[args['location']][r],
                        'world')
            
            poses[args['location']].pop(r)
            
        print('Done')


if __name__ == '__main__':
    main()
