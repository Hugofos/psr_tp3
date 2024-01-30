#!/usr/bin/env python3

from functools import partial
import subprocess
import rospy
from std_msgs.msg import Int32
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from random import randint

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
#from ultralytics_ros.msg import YoloResult

server = None
marker_pos = 0.5
empty_marker = None

menu_handler = MenuHandler()

h_first_entry = 0

def makeBox(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.2

    return marker

def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control

def makeEmptyMarker(dummyBox=True):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker(name):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)

def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher, update_text = True):
    print('BANANA')
    if update_text:
        updateText('Moving to ' + location)

    print('Called moving to ' + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    print('Sending Goal move to ' + location)
    goal_publisher.publish(ps)

    result_msg = rospy.wait_for_message('move_base/result', MoveBaseActionResult)

    if update_text:
        updateText(result_msg.status.text)
    print(result_msg.status.text)

def photograph(feedback, x, y, z, R, P, Y, location, goal_publisher):

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)

    updateText('Taking photograph of ' + location)

    command = ['rosrun', 'psr_tp3', 'photograph.py']
    subprocess.call(command)
    rospy.sleep(1)

    updateText('Photograph of ' + location + ' saved')

def count_spheres_bedroom(feedback, x, y, z, R, P, Y, location, goal_publisher):

    r = randint(1, 3)

    updateText('Spawning spheres in the bedroom')
    command = ['rosrun', 'psr_tp3', 'spawn_object.py', '-q', str(r), '-l', 'bedroom', '-o', 'sphere_v']
    subprocess.call(command)

    count = 0

    command = ['rosrun', 'psr_tp3', 'sphere_detection.py']
    process = subprocess.Popen(command)

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    
    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, -7.713984, 0.336816, -0.001007, -0.000004, 0.003176, 2.477500, location, goal_publisher)

    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Final count: ' + str(count))
    print(count)

    process.terminate

def count_spheres_living_room(feedback, x, y, z, R, P, Y, location, goal_publisher):

    r = randint(1, 3)

    updateText('Spawning spheres in the living_room')
    command = ['rosrun', 'psr_tp3', 'spawn_object.py', '-q', str(r), '-l', 'living_room', '-o', 'sphere_v']
    subprocess.call(command)

    count = 0

    command = ['rosrun', 'psr_tp3', 'sphere_detection.py']
    process = subprocess.Popen(command)

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    
    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 3.685929, -1.113345, -0.001006, -0.000003, 0.003169, -2.691476, location, goal_publisher)

    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 2.973578, -4.053649, -0.001006, -0.000003, 0.003169, -2.691388, location, goal_publisher)

    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Final count: ' + str(count))
    print(count)

    process.terminate

def count_spheres_kitchen(feedback, x, y, z, R, P, Y, location, goal_publisher):

    r = randint(1, 3)

    updateText('Spawning spheres in the kitchen')
    command = ['rosrun', 'psr_tp3', 'spawn_object.py', '-q', str(r), '-l', 'kitchen', '-o', 'sphere_v']
    subprocess.call(command)

    count = 0

    command = ['rosrun', 'psr_tp3', 'sphere_detection.py']
    process = subprocess.Popen(command)

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    
    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 5.948543, -2.964540, -0.001006, -0.000003, 0.003169, 0.302229, location, goal_publisher)

    result_msg = rospy.wait_for_message('spheres/count', Int32)
    count += int(result_msg.data)
    updateText('Final count: ' + str(count))
    print(count)

    process.terminate

def count_cubes(feedback, x, y, z, R, P, Y, location, goal_publisher):

    r = randint(1, 9)

    updateText('Spawning cubes in the house')
    command = ['rosrun', 'psr_tp3', 'spawn_object.py', '-q', str(r), '-o', 'cube_g']
    subprocess.call(command)

    count = 0

    command = ['rosrun', 'psr_tp3', 'cube_detection.py']
    process = subprocess.Popen(command)

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    
    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, -6.486578, -0.466253, -0.001007, -0.000002, 0.003179, 2.286561, location, goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 0.918561, 2.381527, -0.001007, -0.000003, 0.003179, 0.303912, 'living_room', goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 1.667873, 0.452354, -0.001008, -0.000002, 0.003181, -2.295446, 'living_room', goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 3.644693, -3.355822, -0.001008, -0.000003, 0.003183, -1.076885, 'living_room', goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 5.670122, -2.903437, -0.001008, -0.000004, 0.003185, -0.649067, 'kitchen', goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Count: ' + str(count))
    print(count)
    rospy.sleep(3)

    moveTo(feedback, 4.130934, -0.895111, -0.001007, -0.000003, 0.003180, 0.566549, 'kitchen', goal_publisher)

    result_msg = rospy.wait_for_message('cubes/count', Int32)
    count += int(result_msg.data)
    updateText('Final count: ' + str(count))
    print(count)

    process.terminate()

def clean_models(feedback):

    updateText('Cleaning models...')
    command = ['rosrun', 'psr_tp3', 'clean_models.py']
    subprocess.call(command)


def updateText(text):
    empty_marker.controls[0].markers[0].text = text
    server.erase('text_marker')
    server.insert(empty_marker)

    server.applyChanges()

def checkPerson(_, goal_publisher):
    updateText('Searching for person')
    moveTo(_, x = -2.796223, y = -0.787348, z = 0, R = 0, P = 0.0003175, Y = 2.654732, location = 'bedroom', goal_publisher = goal_publisher)
    rospy.sleep(3)
    result_msg = rospy.wait_for_message('yolo_result2', YoloResult)
    if (len(result_msg.detections.detections) < 1):
        moveTo(_, x = 2.244920, y = 1.094999, z = 0, R = 0, P = 0.0003175, Y = -1.692657, location = 'living_room', goal_publisher = goal_publisher)
        rospy.sleep(3)
        result_msg = rospy.wait_for_message('yolo_result2', YoloResult)
    if (len(result_msg.detections.detections) < 1):
        moveTo(_, x = 2.527817, y = 1.706612, z = 0, R = 0, P = 0.0003175, Y = -0.580914, location = 'kitchen', goal_publisher = goal_publisher)
        rospy.sleep(3)
        result_msg = rospy.wait_for_message('yolo_result2', YoloResult)

    if len(result_msg.detections.detections) < 1:
        updateText('Person not found')
    else:
        updateText('Found person')

def main():

    global server, h_first_entry, empty_marker

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    empty_marker = makeEmptyMarker()
    empty_marker.name = 'text_marker'
    empty_marker.pose.position.z = 0.5

    # Create a text marker
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.scale.z = 0.4
    text_marker.color.r = 0.0
    text_marker.color.g = 0.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0
    text_marker.text = "Waiting..."
    text_marker.action = Marker.ADD

    # Add the text marker to the controls of the empty marker
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(text_marker)
    empty_marker.controls.append(control)

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    server = InteractiveMarkerServer("mission")
    print(server)
    
    # Publish the empty marker (which contains the text marker)
    server.insert(empty_marker)

    h_first_entry = menu_handler.insert("Move to")
    h_second_entry = menu_handler.insert("Photograph")
    h_third_entry = menu_handler.insert("Actions")

    entry = menu_handler.insert("kitchen", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("bedroom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("living_room", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=1.213394, y=-0.339148, z=-0.001008,
                                                 R=-0.000014, P=0.003186, Y=-1.569838,
                                                 location='living_room',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("bedroom", parent=h_second_entry,
                                callback=partial(photograph,
                                                 x=-2.787409, y=-1.624719, z=-0.001008,
                                                 R=-0.000003, P=0.003182, Y=2.338958,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))
                                                 
    entry = menu_handler.insert("living_room", parent=h_second_entry,
                                callback=partial(photograph,
                                                 x=1.365698, y=2.405371, z=-0.001008,
                                                 R=-0.000003, P=0.003182, Y=-1.655986,
                                                 location='living_room',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("kitchen", parent=h_second_entry,
                                callback=partial(photograph,
                                                 x=3.449302, y=1.852453, z=-0.001008,
                                                 R=-0.000004, P=0.003181, Y=-0.603640,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("total green cubes", parent=h_third_entry,
                                callback=partial(count_cubes,
                                                 x=-3.613628, y=-0.182284, z=-0.001008,
                                                 R=-0.000002, P=0.003183, Y=1.888727,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("purple spheres bedroom", parent=h_third_entry,
                                callback=partial(count_spheres_bedroom,
                                                 x=-5.641928, y=-0.543833, z=-0.001008,
                                                 R=-0.000012, P=0.003185, Y=1.589689,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("purple spheres living_room", parent=h_third_entry,
                                callback=partial(count_spheres_living_room,
                                                 x=-0.698014, y=2.385116, z=-0.001007,
                                                 R=-0.000003, P=0.003177, Y=1.432500,
                                                 location='living_room',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("purple spheres kitchen", parent=h_third_entry,
                                callback=partial(count_spheres_kitchen,
                                                 x=7.161055, y=-3.045271, z=-0.001006,
                                                 R=-0.000004, P=0.003170, Y=-1.408715,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("clean models", callback=clean_models)

    entry = menu_handler.insert("person", parent=h_third_entry, callback=partial(checkPerson, goal_publisher = goal_publisher))

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
