#!/usr/bin/env python3

from functools import partial
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult

server = None
marker_pos = 0.5

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0


def enableCb(feedback):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible(h_first_entry, True)

    menu_handler.reApply(server)
    rospy.loginfo("update")
    server.applyChanges()


def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
    menu_handler.reApply(server)
    print("DONE")
    server.applyChanges()


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


def deepCb(feedback):
    rospy.loginfo("The deep sub-menu has been found.")


def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher, text_marker_publisher, text_marker):
    text_marker.text = 'Moving to ' + location
    text_marker_publisher.publish(text_marker)
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
    print(result_msg.status.text)

def main():

    global server

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create a publisher for the text marker
    text_marker_publisher = rospy.Publisher('/text_marker', Marker, queue_size=1)

    empty_marker = makeEmptyMarker()
    empty_marker.pose.position.z = 0.5

    # Create a text marker
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.scale.z = 0.4
    text_marker.color.r = 0.0
    text_marker.color.g = 0.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0
    text_marker.text = "Test"
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

    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Move to")

    entry = menu_handler.insert("kitchen", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher, text_marker_publisher = text_marker_publisher, text_marker = text_marker))

    entry = menu_handler.insert("bedroom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher, text_marker_publisher = text_marker_publisher, text_marker = text_marker))

    # entry = menu_handler.insert("living room", parent=h_first_entry, callback=moveToLivingRoom)

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main()
