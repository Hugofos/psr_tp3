#!/usr/bin/env python3
import rospy
import rosservice
from gazebo_msgs.srv import DeleteModel

def main():
    
    rospy.init_node('clean_models', anonymous=True)

    try:
        models = rospy.ServiceProxy('/gazebo/get_world_properties', rosservice.get_service_class_by_name('/gazebo/get_world_properties'))().model_names
    except rospy.ServiceException as e:
        rospy.logerr(str(e))
        return
    
    models_to_delete = {'sphere', 'cube', 'person'}
    count = 0
    for m in models:
        if m.split('_')[0] in models_to_delete:
            try:
                delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                delete_model(m)
                count += 1
            except rospy.ServiceException as e:
                rospy.logerr(str(e))

    rospy.loginfo('Cleaned ' + str(count) + ' models')

if __name__ == '__main__':
    main()    