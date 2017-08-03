/*--
 *      This node compute the traversability from the pointCloud map from voxblox or file
 *      Using only pcl tools
 *
 *      Subscrib :
 *          /voxblox_node/mesh_pointcloud point cloud from voxblox reconstruction
 *
 *      Publish :
 *          /image_traversability the binary image of Traversability
 *          /normals markers to visualize the normals on rviz
 *          /point_cloud_map the map as a point cloud, to isualize on rviz if load from file
 *          /point_cloud_traversability the traversability map as a point cloud
 *
 *      Service :
 *          /load_mesh_srv to trigger the meshCallBack without msg (used when load from file)
 *          /publish_srv to ask for the traversability if the auto_pub param is not true
 *
 *      Parameters :
 *          verbose to show steps, time consuming, image processed
 *          automatic_pub if false you have to call the service to publish info
 *          image_scale_ resolution of the image (pixels/m)
 *          load_from_file if you want to load mesh from file instead of voxblox
 *          z_threshold_ The value of the threshold on traverasbility altitude
 *          file The path to the file you want to use
 *
 *      Approach :
 *          1) convert and filtter entry
 *          2) compute normals using pcl tools
 *          3) compute traversability by thresholding point cloud on slope and altitude
 *          4) filtering traversability images
 */
#include <mesh_to_traversability_pcd/mesh_to_traversability_pcd.hpp>

namespace mesh_to_traversability {

MeshToPCDConverter::MeshToPCDConverter(ros::NodeHandle nh,
                                               ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(kDefaultVerbose),
      load_from_file(kDefaultLoadFromFile),
      automatic_pub_(kDefaultAutomaticPub),
      z_threshold_(kDefaultZThreshold),
      image_scale_(kDefaultImageScale),
      file(kDefaultFile)
{
    // Initial interaction with ROS
    getParametersFromRos();
    subscribeToTopics();
    advertiseTopics();

    // Initialise service
    load_mesh_srv_ = nh_private_.advertiseService("load_mesh_srv", &MeshToPCDConverter::loadMeshCallback, this);
    publish_srv_  = nh_private_.advertiseService("publish_srv", &MeshToPCDConverter::publishCallback, this);
}

void MeshToPCDConverter::subscribeToTopics() {
    mesh_sub_ = nh_.subscribe("mesh", 10, &MeshToPCDConverter::meshCallback, this);
}

void MeshToPCDConverter::advertiseTopics() {
    normals_pub = nh_private_.advertise<visualization_msgs::Marker>("normals", 1, true);
    if(load_from_file || verbose_) pcd_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointNormal>>("point_cloud_map", 1, true);
    traversability_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointNormal>>("point_cloud_traversability", 1, true);

    image_transport::ImageTransport it(nh_);
    img_pub = it.advertise("/image_traversability", 1);
}

void MeshToPCDConverter::getParametersFromRos() {
    nh_private_.param("verbose", verbose_, verbose_);
    nh_private_.param("load_from_file", load_from_file, load_from_file);
    nh_private_.param("automatic_pub", automatic_pub_, automatic_pub_);
    nh_private_.param("z_threshold", z_threshold_, z_threshold_);
    nh_private_.param("image_scale", image_scale_, image_scale_);
    nh_private_.param("file", file, file);
}

// service call back to load a mesh from a file (.pcd or .ply)
bool MeshToPCDConverter::loadMeshCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //const pcl_msgs::PolygonMesh fake_mesh_msg;
    const pcl::PointCloud<pcl::PointXYZRGB> fake_mesh_msg;
    meshCallback(fake_mesh_msg);
    return true;
}

// service call back to compute traversability in the next mesh (when automatic_pub_ = false)
bool MeshToPCDConverter::publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    pub_ = true;
}

void MeshToPCDConverter::meshCallback(const pcl::PointCloud<pcl::PointXYZRGB>& mesh_msg)//const pcl_msgs::PolygonMesh& mesh_msg)
{
    if(!automatic_pub_ && !pub_) return; // no need to compute the traversability if you don't send it

    if (verbose_) ROS_INFO("Mesh received, starting treatment.");
    ros::Time time0, time1, time2;
    double duration;
    time0 = ros::Time::now();

    /// Converting from message to an object
    time1 = ros::Time::now();
    pcl::PolygonMesh polygon_mesh;
    //pcl_conversions::toPCL(mesh_msg, polygon_mesh);
    if(load_from_file)ROS_INFO_STREAM("From file :"<<file<<" - file type : "<<fileType(file));
    if(load_from_file && fileType(file) == "ply") pcl::io::loadPolygonFilePLY (file, polygon_mesh);

    /// Load input mesh into a PointCloud<T> with an appropriate type
    if (verbose_) ROS_INFO("Convertion in pointCloud");
    pcl::PCLPointCloud2 cloud_blob;
    if(load_from_file && fileType(file) == "pcd") pcl::io::loadPCDFile (file, cloud_blob);
    else cloud_blob = polygon_mesh.cloud;
    // convert ros::pointcloud2 to pcl::pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(load_from_file) pcl::fromPCLPointCloud2 (cloud_blob, *cloud); // the data is available in cloud
    else pcl::copyPointCloud(mesh_msg,*cloud);

    /// remove unecessary points
    pcl::PointIndices::Ptr indices_in (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (false); // Initializing with true will allow us to extract the removed indices
    // clean from outliers
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      pcl::PointXYZ point = cloud->points[i];
      if(point.data[2]>-0.6 && point.data[2] < z_threshold_) // point too low (under the ground)
          indices_in->indices.push_back(i);
    }
    eifilter.setIndices(indices_in);
    eifilter.filterDirectly(cloud);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    if(cloud->points.size() < 100) return;

    //---------------------------------------- treatment of the point cloud ---------------------------------------------------//
    /// Normal estimation
    if (verbose_) ROS_INFO("Normals computation");
    time1 = ros::Time::now();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (120);
    // set the view point far from the ground to have the normals pointing to the top
    n.setViewPoint (cloud->points[0].data[0], cloud->points[0].data[1], cloud->points[0].data[2]+10.0);
    n.compute (*normals); // normals now contain the point normals

    /// Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals); // cloud_with_normals = Pointcloud + normals
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Treate pointCloud data to extract feasible path
    time1 = ros::Time::now();
    if (verbose_) ROS_INFO("Feasible path extraction");
    pcl::ExtractIndices<pcl::PointNormal> eifilter2 (false); // Initializing with true will allow us to extract the removed indices
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_freespace (new pcl::PointCloud<pcl::PointNormal>);
    // create traversability pointcloud from segmentation on normal_z
    pcl::copyPointCloud(*cloud_with_normals,*cloud_freespace);
    pcl::PointIndices::Ptr indices_traversability (new pcl::PointIndices());
    for (int i = 0; i < cloud_freespace->points.size(); ++i)
    {
      pcl::PointNormal point = cloud_freespace->points[i];
      // segmentation from steep and altitude
      if(point.normal[2] > 0.9 && point.data[2] < z_threshold_ )
          indices_traversability->indices.push_back(i);
    }
    eifilter2.setIndices(indices_traversability);
    eifilter2.filterDirectly(cloud_freespace);
    // Remove Nan in pointCloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_freespace,*cloud_freespace,indices);
    pcl::removeNaNNormalsFromPointCloud(*cloud_freespace,*cloud_freespace,indices);
    // remove isolated points
    pcl::RadiusOutlierRemoval<pcl::PointNormal> rorfilter (false); // Initializing with true will allow us to extract the removed indices
    rorfilter.setInputCloud (cloud_freespace);
    rorfilter.setRadiusSearch (0.1);
    rorfilter.setMinNeighborsInRadius (4);
    rorfilter.setNegative (false);
    rorfilter.filter (*cloud_freespace);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

//--------------------- Publishing ---------------------------------------------------//
    /// Creating and Publishing normals markers
    time1 = ros::Time::now();
    if (verbose_) ROS_INFO("Creation and Publishing normals markers");
    ros::Time time = ros::Time::now();
    scale_ = 0.2;
    lineWidth_ = 0.02;
    color_.r = 0.8;
    color_.g = 0.2;
    color_.b = 0.2;
    color_.a = 1.0;
    marker_.ns = "vector";
    marker_.lifetime = ros::Duration();
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.type = visualization_msgs::Marker::LINE_LIST;
    marker_.scale.x = lineWidth_;
    if(load_from_file) marker_.header.frame_id = "map";
    else marker_.header.frame_id = mesh_msg.header.frame_id;
    marker_.header.stamp.fromNSec(time.toSec());
    marker_.points.clear();
    marker_.colors.clear();
    for (size_t i = 0; i < cloud_with_normals->points.size(); ++i)
    {
      pcl::PointNormal point = cloud_with_normals->points[i];
      geometry_msgs::Vector3 vector;
      vector.x = point.normal[0];
      vector.y = point.normal[1];
      vector.z = point.normal[2];

      if(vector.x<1000 && vector.y<1000 && vector.z<1000 && vector.x>-1000 && vector.y>-1000 && vector.z>-1000)
      {
          geometry_msgs::Point startPoint;
          startPoint.x = point.data[0];
          startPoint.y = point.data[1];
          startPoint.z = point.data[2];
          marker_.points.push_back(startPoint);

          geometry_msgs::Point endPoint;
          endPoint.x = startPoint.x + scale_ * vector.x;
          endPoint.y = startPoint.y + scale_ * vector.y;
          endPoint.z = startPoint.z + scale_ * vector.z;
          marker_.points.push_back(endPoint);

          marker_.colors.push_back(color_); // Each vertex needs a color.
          marker_.colors.push_back(color_);
      }
    }
    normals_pub.publish(marker_);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Publishing PointCloud
    // Original map
    if (verbose_) ROS_INFO("Publishing Point Cloud");
    time1 = ros::Time::now();
    if(load_from_file || verbose_)
    {
        if(load_from_file )cloud_with_normals->header.frame_id="map";
        else cloud_with_normals->header.frame_id = mesh_msg.header.frame_id;
        pcd_pub_.publish(*cloud_with_normals);
    }
    // Traversability map
    if(load_from_file) cloud_freespace->header.frame_id="map";
    else cloud_freespace->header.frame_id = mesh_msg.header.frame_id;
    traversability_pub_.publish(*cloud_freespace);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Make traversability into image
    if (verbose_) ROS_INFO("Compution of traversability image");
    time1 = ros::Time::now();
    // find limits of point cloud
    double xmin = 1000;
    double xmax =-1000;
    double ymin = 1000;
    double ymax =-1000;
    for (size_t i = 0; i < cloud_freespace->points.size(); ++i)
    {
        pcl::PointNormal point = cloud_freespace->points[i];
        if( point.data[0] < xmin) xmin = point.data[0];
        if( point.data[0] > xmax) xmax = point.data[0];
        if( point.data[1] < ymin) ymin = point.data[1];
        if( point.data[1] > ymax) ymax = point.data[1];
    }
    double scale = image_scale_; // resolution of the image of traversability pixels/m
    int width = scale*(xmax-xmin)+1;
    int height = scale*(ymax-ymin)+1;
    if (verbose_) ROS_INFO_STREAM("zone : "<<xmin<<"-"<<xmax<<" , "<<ymin<<"-"<<ymax<<" -> scale="<<scale<<" ("<<width<<","<<height<<")");
    cv::Mat traversability_img = cv::Mat::zeros(height, width, CV_8UC1);
    // projection of the traversability point into the image
    for (size_t i = 0; i < cloud_freespace->points.size(); ++i)
    {
        pcl::PointNormal point = cloud_freespace->points[i];
        cv::circle(traversability_img,cv::Point(width-(unsigned int)(scale*(point.data[0]-xmin)), (unsigned int)(scale*(point.data[1]-ymin))),2,255,-1);
    }
    //cv::imshow("traversability_image_raw",traversability_img);
    // median filter to remove small black pixels
    cv::medianBlur(traversability_img,traversability_img,5);
    // erode to make bigger the places to evode
    int element_size = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size + 1, 2*element_size+1 ), cv::Point( element_size, element_size ) );
    cv::morphologyEx( traversability_img, traversability_img, cv::MORPH_ERODE, element ); // on enlève les moyenne taches blanche dans le noir
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// publishing
    sensor_msgs::ImagePtr msgPublish;
    msgPublish = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_img).toImageMsg();
    img_pub.publish (msgPublish);

    if (verbose_) ROS_INFO("Treatment of the mesh ok!");
    time2 = ros::Time::now();
    duration = time2.toSec() - time0.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec\n");
    if (verbose_) cv::waitKey(50); //wait for cv::imshow to appear
}


//-------------- Methodes usefull ---------------------------//
std::string fileType(std::string fichier)
{
    // cut on "/" and we take the last part
    std::vector<std::string> tab;
    boost::split(tab, fichier, boost::is_any_of("/"));
    std::string nameExtension = tab[tab.size()-1];
    // we read the extension by cut the name on the dot
    tab.clear();
    boost::split(tab, nameExtension, boost::is_any_of("."));
    std::string type = tab[1];

    return type;
}


}  // namespace mesh_to_grid_map
