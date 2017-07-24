# mesh_to_traversability_image #
ROS package containing 1 node :

*  **mesh_to_traversability_image_node :** 

## Dependences ##

* Opencv, pcl, voxblox

## Usage ##

* **roslaunch mesh_to_traversability_image.launch :** 

Launch rosbag (you have to use yours) 

dense_stereo (you have to use your calibration file .yaml)

mesh_to_traversability_image_node

* **roslaunch pcd_to_traversability_image.launch :** 

open a .pcd or .ply file to compute traversability using mesh_to_traversability_image_node

## Parameters ##

* **load_from_file :** true to load from file, false to connect to voxblox mesh

* **file :** the file to open

* **image_scale :** resolution of the image pixels/m

* **z_threshold :** if you want to threshold the altitude of the traversability

* **automatic_pub :** if false, you need to use service call for the publishment

