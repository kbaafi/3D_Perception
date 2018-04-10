![image1]: ./images/confusion.png "Confusion Matrix"
![image2]: ./images/objects_white.png "Object Clusters"
![image3]: ./images/pt.png "Pass-through filtering"
![image4]: ./images/ransac.png "RANSAC Plane Segmentation"
![image5]: ./images/stat_out.png "Statistical Outlier Removal"
![image6]: ./images/vox.png "Vox Downsampling"
![image7]: ./images/world_1.png "Labeled Objects in World 1"
![image8]: ./images/world_2.png "Labeled Objects in World 2"
![image9]: ./images/world_3.png "Labeled Objects in World 3"
![image10]: ./images/worldpc.png "Direct Camera Feed"
![image11]: ./images/world_3_cloud.png "Cloud of detected objects in World 3"

# 3D Perception Pipeline
In this project we attempt to identify objects from their RGBXYZ representations encoded from an RGBD camera. As part of a larger pick and place / categorize project, we need our robot to identify certain objects and decide in which box to place them. This project currently only focuses on the perception portion of the problem. We present a Linear SVM classifier which given the RBGD representations of the pretrained objects is able to classify them to an accuracy of 87.8%, which is enough to prevent overfitting. 

# Training the Classifier
In building the classifier the following steps were taken. 
For each image:
1. capture point cloud of the image in different spatial(x,y,z) orientations
2. compute the normalized histogram of the color data of the object
3. compute the normalized histogram of the surface normals of the object
4. concatenate the histograms and label the feature

A rosnode is designed to generate the features. Run:
```sh
$ rosrun sensor_stick capture_features_pr2.py
```
The code for the feature extraction node can be found in 
```sh
/sensor_stick/scripts/capture_features_pr2.py 
```

## The SVM Classifier
The labeled features are trained on a linear SVM with 10 fold cross validation. For each image we generate 30 datapoints and this gives us a total of 240 labeled datapoints for the 8 different objects. The resulting confusion matrix is presented below:
[image1]

For training run:
```sh
$ rosrun sensor_stick train_svm_pr2.py              and then in a new terminal
$ roslaunch sensor_stick training.launch
```
You can find the model training node at `/sensor_stick/scripts/train_svm_pr2.py`

This generates the `model_pr2.sav` file which has to be copied into `~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts/`

# Object Detection

### Preprocessing Camera Feed
Given the camera feed from the robot, the following tasks were performed to segment the point cloud and classify each segment as an object. 

### 1. Statistical Outlier Removal
The original camera data is shown below:
[image10]

As can be seen obtained image from the camera is noisy. Using statistical outlier filtering, we can remove said noise to yeild a more usable input representation. For this we use the 
```python
    pcl.make_statistical_outlier_filter()
```
The number of points from which the outlier is determined is `20` and the threshold scale factor is `0.1`

The resulting point cloud is shown below:
[image5]

### Voxel Downsampling
The resulting point cloud filter is then downsampled using a voxel filter with leaf size of `0.01`. The result is shown below:

### Passthrough Filtering
Next we extract our region of interest from the cloud. In the `z-axis` we use the boundaries `z = 0.61 and z = 0.95`. In the `y-axis` we use the boundaries `y = -0.4 and y = 0.4`. The result is shown below:
[image6]

### RANSAC Plane Segmentation
Having obtained our region of interest, we can now remove the plane that makes the table from the scene, ensuring that we have removed as much of the surrounding as possible. We setup a RANSAC plane model with a distance threshold of 0.01 to remove the remaining table data. This ensures that we only have the objects that we are targeting
[image4]

### Clustering filtered cloud with DBSCAN
The below shows the effect of clustering on the RANSAC segmented cloud. As you can see from the whitened point cloud, we are able to retrieve as much of the target objects as possible
[image2]
# Object Detection and Labeling
For each object obtained in the preceeding step, we can now perform feature extraction and classification using the same methods described in the training phase. There are three working environments and for each environment, we output the list of detected objects and the locations into which they are to be placed. The below shows the results of our 3D object detector

World 1
[image7]
World 2
[image8]
World 3
[image9][image11]


