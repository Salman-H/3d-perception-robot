#!/usr/bin/env python
"""
Module for training a model on object features defined in features.py.

Models for objects of interest can be found in the models folder.

"""

__author__ = 'Salman Hashmi'


import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    """Extract normals from input cloud."""
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


def photograph_models(models, shots):
    """Take pictures of each object model in tranining set."""
    TRIES = 5
    print('\nnumber of shots per model: {}'.format(shots))
    for model_name in models:
        print('\nmodel name: {}'.format(model_name))
        spawn_model(model_name)
        # For each object model, take number of pictures equal to shots
        for i in range(shots):
            print('shot number: {}'.format(i))
            good_sample = False
            try_count = 0
            # For each model, make number of attempts equal to TRIES to get a
            # valid point cloud then skip to next model
            while not good_sample and try_count < TRIES:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
                # Check for invalid clouds
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud sample')
                    try_count += 1
                else:
                    print('valid cloud sample')
                    good_sample = True
            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])
        # Delete spawned model after taking its shot
        delete_model()


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
        'biscuits',
        'soap',
        'soap2',
        'book',
        'glue',
        'sticky_notes',
        'snacks',
        'eraser']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    # Take pictures of each object model of interest with RGBD camera
    NUM_SHOTS = 1000
    photograph_models(models, NUM_SHOTS)

    # Construct training set
    pickle.dump(labeled_features, 
                open('./src/RoboND-Perception-Project/training_set.sav', 'wb'))
