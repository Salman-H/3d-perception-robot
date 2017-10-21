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


def passthrough_filter(cloud):
    """Apply a pass through filter."""
    # Create a passthrough filter object
    passthrough = cloud.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    passthrough.set_filter_field_name('z')
    axis_min = 0.76
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    # Return the resultant point cloud
    return passthrough.filter()


def plane_segmentation(cloud):
    """Apply a RANSAC plane segmentation."""
    # Create a segmentation object
    seg = cloud.make_segmenter()
    # Set the desired model: PLANE chosen to segment the table out
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model (a plane)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Return the set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    return inliers, coefficients


def euclidean_clustering(object_cloud):
    """Apply Euclidean clustering to segment cloud into individual objects."""
    # PCL's Euclidean Clustering algorithm requires a point cloud with only
    # spatial information
    white_cloud = XYZRGB_to_XYZ(object_cloud)
    # The k-d tree data structure is used in the Euclidian Clustering algorithm
    # to decrease the computational burden of searching for neighboring points.
    # While other efficient algorithms/data structures for nearest neighbor search
    # exist, PCL's Euclidian Clustering algorithm only supports k-d trees.
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold as well as min/max cluster size (in points)
    # These parameters are to be tweaked to find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(1700)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud


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
    
    # Apply a pass through filter
    cloud_filtered = passthrough_filter(cloud_filtered)

    # Apply RANSAC Plane Segmentation to obtain inlier indices and model coeff.
    inliers, coefficients = plane_segmentation(cloud_filtered)

    # Extract inliers and outliers
    # negative=False: extract only the subset of points that fit model (inliers)
    # negative=True: extract subset of points that did not fit the model (outliers)
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)

    # Apply Eculidian clustering on extracted outliers (table objects)
    cluster_cloud = euclidean_clustering(extracted_outliers)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

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

    # Create Publishers to publish point cloud data for the table and objects
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    # Prevents node from exiting until an intentional shutdown is invoked
    while not rospy.is_shutdown():
        rospy.spin()
