#ifndef MESH_TO_TRAVERSABILITY_MAP_H
#define MESH_TO_TRAVERSABILITY_MAP_H

#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl_msgs/PolygonMesh.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace mesh_to_traversability {

std::string fileType(std::string fichier);
constexpr bool kDefaultVerbose = false;
constexpr bool kDefaultLoadFromFile = false;
constexpr bool kDefaultAutomaticPub = false;
static double kDefaultImageScale = 20.0;
static double kDefaultZThreshold = 100.0;
static const std::string kDefaultFile = "/home/eth/mesh.pcd";

class MeshToPCDConverter {

public:
  bool load_from_file;
  MeshToPCDConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);
  // service callBack
  bool loadMeshCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // service CallBak to load mesh from a file
  bool publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  // Msgcallback
  //void meshCallback(const pcl_msgs::PolygonMesh& mesh);
  void meshCallback(const pcl::PointCloud<pcl::PointXYZRGB>& mesh);

private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber mesh_sub_;
  // Data publisher
  ros::Publisher normals_pub;
  ros::Publisher pcd_pub_;
  ros::Publisher traversability_pub_;
  image_transport::Publisher  img_pub;
  // services
  ros::ServiceServer load_mesh_srv_;
  ros::ServiceServer publish_srv_;
  ros::ServiceClient client;

  // Marker to be published.
  visualization_msgs::Marker marker_;
  // Types that are transformed to vectors.
  std::vector<std::string> types_;
  // Type that is the position of the vectors.
  std::string positionLayer_;
  // Scaling of the vectors.
  double scale_;
  // Width of the line markers [m].
  double lineWidth_;
  // Color of the vectors.
  std_msgs::ColorRGBA color_;

  // Params
  bool verbose_;
  bool automatic_pub_;
  bool pub_;
  std::string file;
  double z_threshold_;
  double image_scale_;
};

} // namespace mesh_to_grid_map

#endif //
