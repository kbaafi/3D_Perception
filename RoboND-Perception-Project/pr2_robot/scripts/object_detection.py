#!/usr/bin/env python

# Import modules
import pcl

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

from geometry_msgs.msg import Pose

MODULE_FILE_NAME = 'model_pr2.sav'
labeled_centroids = []

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def extract_cloud_features(cloud, color_bins, normals_bins):
    chists = compute_color_histograms(cloud, bin_size = color_bins, using_hsv=True)
            
    normals = get_normals(cloud)
    nhists = compute_normal_histograms(normals, bin_size = normals_bins)
    
    feature = np.concatenate((chists, nhists))

    return feature

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

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(20)

    # Set threshold scale factor
    x = 0.1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_outlier_filtered = outlier_filter.filter()
    
    # TODO: Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_outlier_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    vox_filtered = vox.filter()

    # TODO: PassThrough Filtering
    # along the z-axis
    passthrough = vox_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.61
    axis_max = 0.95
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    ptz_filtered = passthrough.filter()

    # Along the y-axis
    passthrough = ptz_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max =  0.4
    passthrough.set_filter_limits(axis_min, axis_max)
    pt_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = pt_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract outliers (what we're interested in)
    cloud_objects = pt_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    # remove color data
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    
    # extract KD-tree from cloud
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(1000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                            white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_objects         = pcl_to_ros(cloud_objects)
    ros_cluster_cloud   = pcl_to_ros(cluster_cloud)
    ros_of              = pcl_to_ros(cloud_outlier_filtered)
    ros_vox             = pcl_to_ros(vox_filtered)
    ros_pt              = pcl_to_ros(pt_filtered)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    pcl_of_pub.publish(ros_of)
    pcl_vox_pub.publish(ros_vox)
    pcl_pt_pub.publish(ros_pt)

    # Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []
    labeled_centroids = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        pcl_cluster_ros = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        color_bins = 32
        normals_bins = 2
        feature = extract_cloud_features(pcl_cluster_ros, color_bins, normals_bins)

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cluster_ros
        detected_objects.append(do)

        #rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
        detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    ####################################
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    world_id = rospy.get_param("world_id")
    yaml_out = []
    

    # TODO: Get/Read parameters
    object_list_param   = rospy.get_param("/object_list")
    boxes_param     = rospy.get_param("/dropbox")

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for o in object_list_param:
        name = o["name"]
        group = o["group"]

        oid = None

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for i,obj in enumerate(object_list):
            if name!= obj.label:
                continue
            else:
                oid = i

            # TODO: Create 'place_pose' for the object
            ros_world       = Int32()
            selected_object = String()

            selected_object.data = name
            ros_world.data = world_id

            selected_robo_arm = String()
            # TODO: Assign the arm to be used for pick_place
            if group=='green':
                selected_robo_arm.data = 'right'
            else:
                selected_robo_arm.data = 'left'

            pick_pose = Pose()
            
            object_cloud_pcl    = ros_to_pcl(obj.cloud)
            pt_array            = object_cloud_pcl.to_array()

            centroid = np.mean(pt_array,axis=0)[:3]
            centroid_float = [np.asscalar(x) for x in centroid]


            pick_pose.position.x = centroid_float[0]
            pick_pose.position.y = centroid_float[1]
            pick_pose.position.z = centroid_float[2]

            selected_box_pose = [0,0,0]
            for elem in boxes_param:
                if (elem['name']== selected_robo_arm.data):
                    selected_box_pose = elem['position']
                    break

            place_pose = Pose()
            place_pose.position.x = selected_box_pose[0]
            place_pose.position.y = selected_box_pose[0]
            place_pose.position.z = selected_box_pose[0]

            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_out.append(make_yaml_dict(ros_world, selected_robo_arm, selected_object, pick_pose, place_pose))
            del object_list[oid]
            #break # move on to next object in params list
            '''
            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request
                resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            '''

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_%i.yaml' % world_id,yaml_out)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_detection', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub         = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub         = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    pcl_of_pub              = rospy.Publisher("/pcl_outlier", PointCloud2, queue_size=1)
    pcl_vox_pub              = rospy.Publisher("/pcl_vox", PointCloud2, queue_size=1)
    pcl_pt_pub              = rospy.Publisher("/pcl_pt", PointCloud2, queue_size=1)
    object_markers_pub      = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub    = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open(MODULE_FILE_NAME, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
