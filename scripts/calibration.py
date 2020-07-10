#!/usr/bin/env python
import rospy
from opencv_object_tracking.msg import position_publish
from opencv_object_tracking.srv import *
from geometry_msgs.msg import Point
import numpy as np
import threading

# define the transfer function from quaternion to rotate matrix
def quaternion_to_rotation_matrix(quat):
  q = quat.copy()
  n = np.dot(q, q)
  if n < np.finfo(q.dtype).eps:
    return np.identity(4)
  q = q * np.sqrt(2.0 / n)
  q = np.outer(q, q)
  rot_matrix = np.array(
    [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], 0.0],
     [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], 0.0],
     [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
     [0.0, 0.0, 0.0, 1.0]],
    dtype=q.dtype)
  return rot_matrix

# handle the loop problem caused by rospy.spin()
def thread_job():
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped

def Handle_Transfer_Object(req):
    # the server to answer the client resquest to send the actual object position in robot base
    global actual_1_position, actual_2_position, actual_1_position_temp, actual_2_position_temp
    actual_1_position.x = actual_1_position_temp[0]
    actual_1_position.y = actual_1_position_temp[1]
    actual_1_position.z = actual_1_position_temp[2]
    actual_2_position.x = actual_2_position_temp[0]
    actual_2_position.y = actual_2_position_temp[1]
    actual_2_position.z = actual_2_position_temp[2]
    actual_position = [actual_1_position, actual_2_position]
    return TransferObjectResponse(actual_position)

def callback(data):
    global object_1_position, object_2_position, counter_1, counter_2
    # store the sample data until it is up to 5 and compute the average position to filter the noise
    if data.counter == 1:
        object_1_position_temp[counter_1,:] = np.array([data.Position_XYZ[0].x, data.Position_XYZ[0].y, data.Position_XYZ[0].z])
        counter_1 = counter_1 + 1
    elif data.counter == 2:
        object_2_position_temp[counter_2,:] = np.array([data.Position_XYZ[0].x, data.Position_XYZ[0].y, data.Position_XYZ[0].z])
        counter_2 = counter_2 + 1
    else: 
        rospy.loginfo("data gap %s", data.counter)
    if counter_1 == 5:
        object_1_position = np.mean(object_1_position_temp, axis=0)
        counter_1 = 0
    if counter_2 == 5:
        object_2_position = np.mean(object_2_position_temp, axis=0)
        counter_2 = 0
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.Position_XYZ)
     
def listener():
    global rotate_matrix, object_1_position, object_2_position, actual_1_position_temp, actual_2_position_temp

    rospy.init_node('listener', anonymous=True)

    # handle the loop problem caused by rospy.spin()
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()

    rospy.Subscriber("position_object", position_publish, callback)
    rospy.Service('transfer_acutal_position', TransferObject, Handle_Transfer_Object)
    while not rospy.is_shutdown():    
        # judge the oject_position whether is empty if not compute the object position in robot base
        if np.where(object_1_position!=0)[0].shape[0]==0 or np.where(object_2_position!=0)[0].shape[0]==0:
            pass
        else:
            actual_1_position_temp = np.dot(rotate_matrix, np.concatenate((object_1_position, [1])))
            actual_2_position_temp = np.dot(rotate_matrix, np.concatenate((object_2_position, [1])))

 
if __name__ == '__main__':
    # get the eye_on_base calibration matrix parameters from master
    translation_x = rospy.get_param('/translation/x')
    translation_y = rospy.get_param('/translation/y')
    translation_z = rospy.get_param('/translation/z')
    rotation_x = rospy.get_param('/rotation/x')
    rotation_y = rospy.get_param('/rotation/x')
    rotation_z = rospy.get_param('/rotation/x')
    rotation_w = rospy.get_param('/rotation/x')
    # get the rotation and translation of the matrix
    quaternion = np.array([rotation_w, rotation_x, rotation_y, rotation_z])
    rotate_matrix = quaternion_to_rotation_matrix(quaternion)
    rotate_matrix[0,3] = translation_x
    rotate_matrix[1,3] = translation_y
    rotate_matrix[2,3] = translation_z
    # count the sample number for average the data
    counter_1 = 0
    counter_2 = 0

    #store the sample data and the final data
    object_1_position_temp = np.zeros((5,3))
    object_2_position_temp = np.zeros((5,3))
    object_1_position = np.zeros(3)
    object_2_position = np.zeros(3)
    #store the sample data and the final data
    actual_1_position_temp = np.zeros(3)
    actual_2_position_temp = np.zeros(3)
    actual_1_position = Point()
    actual_2_position = Point()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

