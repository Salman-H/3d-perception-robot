#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def outlier_removal_filter(cloud):
    """Apply a statistical outlier removal filter."""
    # Make a filter object
    outlier_filter = cloud.make_statistical_outlier_filter()
    NEIGHBORS = 50
    THRESHOLD = 1.0
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(NEIGHBORING_POINTS)
    # Any point with a mean distance larger than global
    # (mean distance+THRESHOLD*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(THRESHOLD)
    # Call the filter function too apply filter
    return outlier_filter.filter()


def voxel_downsampling(cloud):
    """Apply voxel grid downsampling."""
    # Create a VoxelGrid filter object for the input point cloud
    vox = cloud.make_voxel_grid_filter()
    # Set the voxel (or leaf) size
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Return the resultant downsampled point cloud
    return = vox.filter()


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    # To process camera point cloud, need to convert it from received ROS 
    # message (in PointCloud2 fromat) to a PCL format for python-pcl.
    cloud = ros_to_pcl(pcl_msg)
    
    # Apply Statistical Outlier Filtering
    cloud_filtered = outlier_removal_filter(cloud)
    
    # Apply Voxel Grid Downsampling
    cloud_filtered = voxel_downsampling(cloud_filtered)
    
    # TODO: PassThrough Filter

    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    # Initialize a new ROS node called perception
    rospy.init_node('perception', anonymous=True)

    # Subsribe the newly initialized node to the camera data (point cloud) 
    # topic "/pr2/world/points". Anytime a pcl message arrives, the message
    # data (a point cloud) from the PR2 robot will be passed to the 
    # pcl_callback() function for processing.
    pcl_sub = rospy.Subscriber("/pr2/world/points",
                                pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    # Prevents node from exiting until an intentional shutdown is invoked
    while not rospy.is_shutdown():
        rospy.spin()
