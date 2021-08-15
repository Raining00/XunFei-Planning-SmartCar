#include "laser_detection/laser_detection.h"

void LaserDetection::LoadMap() {
  ros::ServiceClient static_map_srv_ = nh.serviceClient<nav_msgs::GetMap>(static_map_topic_);
  ros::service::waitForService(static_map_topic_, -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if (static_map_srv_.call(req, res)) {
    ROS_INFO("Received Static Map");
    cv::Mat grid_map = cv::Mat(res.map.info.height, res.map.info.width, CV_8U, const_cast<int8_t *>(&res.map.data[0]), (size_t) res.map.info.width);
    cv::Mat detected_edges;
    cv::Canny(grid_map, detected_edges, 100, 200);
    // cv::imshow("edge_map", detected_edges);
    // cv::waitKey(0);
    std::vector<pcl::PointXYZ> edge_points;
    for (int h_i = 0; h_i < detected_edges.cols; h_i++) {
      for (int w_j = 0; w_j < detected_edges.rows; w_j++) {
        cv::Scalar pixel = detected_edges.at<uchar>(w_j, h_i);
        if (pixel.val[0] == 255) {
          pcl::PointXYZ p;
          p.x = res.map.info.origin.position.x + float(h_i) * res.map.info.resolution;
          p.y = res.map.info.origin.position.y + float(w_j) * res.map.info.resolution;
          p.z = 0.05;
          edge_points.push_back(p);
        } else {
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZ> map_points;
    map_points.width = edge_points.size();
    map_points.height = 1;
    map_points.is_dense = false;
    map_points.points.resize(map_points.width * map_points.height);
    for (size_t i = 0; i < edge_points.size(); i++) { map_points.points[i] = edge_points[i]; }
    *map_cloud = map_points;
    // for debug
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // *cloud = map_points;
    // pcl::visualization::CloudViewer viewer("map points");
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped()) {}
    // } else {
    //   ROS_ERROR("Get static map failed");
    //   return;
    // }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(map_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*downsample_map);
  *map_cloud = *downsample_map;
  ROS_INFO("Load pointcloud map successfully!");
}

void LaserDetection::UpdateLaser(const sensor_msgs::LaserScanConstPtr &msg) {
  laser_geometry::LaserProjection projector_;
  // update
  // system("clear");
  tf::StampedTransform transform_laser;
  try {
    listener_.lookupTransform(map_frame_, laser_frame_, ros::Time(0), transform_laser);
  } catch (tf::TransformException ex) { return; }
  // std::printf("received laser \n");
  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud(laser_frame_, *msg, cloud, listener_);
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, pc2);
  pcl::fromROSMsg(pc2, cloud_out);
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  *laser_cloud = cloud_out;

  double roll, pitch, yaw;
  transform_laser.getBasis().getEulerYPR(yaw, pitch, roll);
  double odom_x, odom_y;
  odom_x = transform_laser.getOrigin().getX();
  odom_y = transform_laser.getOrigin().getY();
  self_pose_.x = odom_x;
  self_pose_.y = odom_y;
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
  transform_(0, 0) = cos(yaw);
  transform_(0, 1) = -sin(yaw);
  transform_(1, 0) = sin(yaw);
  transform_(1, 1) = cos(yaw);
  transform_(0, 3) = odom_x;
  transform_(1, 3) = odom_y;
  pcl::transformPointCloud(*laser_cloud, *odom_cloud, transform_);
  Detection(odom_cloud);
}

void LaserDetection::Detection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser) {
  // match two cloud
  // std::printf("detecting object \n");
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-8);
  icp.setInputSource(laser);
  icp.setInputTarget(map_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f Ti_inv = Eigen::Matrix4f::Identity();
  Ti = icp.getFinalTransformation();
  Ti_inv = Ti.inverse();
  *sub_in = Final;
  // std::printf("completed icp \n");
  // outlier
  pcl::SegmentDifferences<pcl::PointXYZ> seg_sub;
  seg_sub.setInputCloud(sub_in);
  seg_sub.setTargetCloud(map_cloud);
  seg_sub.setDistanceThreshold(segment_distance_threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc(new pcl::PointCloud<pcl::PointXYZ>);
  seg_sub.segment(*sub_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*sub_pc, *outlier_pc, Ti_inv);
  // publish outlier pc
  PublishSegmentPointcloud(outlier_pc);
  if (show_inlier_pc) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_map_pc(new pcl::PointCloud<pcl::PointXYZ>);
    if (!sub_pc->points.empty()) {
      // inlier
      pcl::SegmentDifferences<pcl::PointXYZ> seg_in;
      seg_in.setInputCloud(sub_in);
      seg_in.setTargetCloud(sub_pc);
      seg_in.setDistanceThreshold(segment_distance_threshold);
      seg_in.segment(*in_map_pc);
      pcl::transformPointCloud(*in_map_pc, *in_map_pc, Ti_inv);
      PublishInMapPointcloud(in_map_pc);
    } else {
      pcl::transformPointCloud(*sub_in, *in_map_pc, Ti_inv);
      PublishInMapPointcloud(in_map_pc);
    }
    printf("in_map_pc size: %zu, outlier_pc size : %zu\n", in_map_pc->points.size(), outlier_pc->points.size());
  } else {
    printf("outlier_pc size : %zu\n", outlier_pc->points.size());
  }
  // cluster
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(sub_pc);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance); // 2cm
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(sub_pc);
  ec.extract(cluster_indices);
  // std::cout << "detect: " << std::endl;
  std::vector<Point> detected_points;
  markers.points.clear();
  markers.colors.clear();
  markers.action = visualization_msgs::Marker::ADD;
  for (auto &cluster_index : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    float center_x, center_y;
    double distance;
    center_x = 0.0;
    center_y = 0.0;
    for (int &index : cluster_index.indices) {
      cloud_cluster->points.push_back(sub_pc->points[index]);
      center_x += sub_pc->points[index].x;
      center_y += sub_pc->points[index].y;
    }
    int cluster_size = cloud_cluster->points.size();
    center_x = center_x / float(cluster_size);
    center_y = center_y / float(cluster_size);
    distance = sqrt(pow((center_x - self_pose_.x), 2) + pow((center_y - self_pose_.y), 2));
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = false;
    // std::cout << "cluster " << j << " x: " << center_x << " y: " << center_y << " width: " << cloud_cluster->width << " dis: " << distance
    //           << std::endl;
    geometry_msgs::Point pt;
    std_msgs::ColorRGBA C;
    pt.x = center_x;
    pt.y = center_y;
    if (distance > max_detection_threshold) {
      C.b = 0.5f;
      C.r = 1.0f;
      C.a = 1.0;
      pt.z = 0.8;
    } else if (distance < min_detection_threshold) {
      C.b = 0.5f;
      C.r = 1.0f;
      C.a = 1.0;
      pt.z = 0.6;
    } else {
      C.g = 0.5f;
      C.r = 1.0f;
      C.a = 1.0;
      pt.z = 0.7;
    }
    markers.colors.push_back(C);
    markers.points.push_back(pt);
  }
  marker_pub_.publish(markers);
  markers.action = visualization_msgs::Marker::DELETEALL;
  // std::cout << std::endl;
}

void LaserDetection::PublishSegmentPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_laser) {
  sensor_msgs::PointCloud2Ptr pub_seg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*seg_laser, *pub_seg);
  pub_seg->header.frame_id = "map";
  pub_seg->header.stamp = ros::Time::now();
  seg_pc_pub_.publish(*pub_seg);
}
void LaserDetection::PublishInMapPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_laser) {
  sensor_msgs::PointCloud2Ptr pub_seg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*seg_laser, *pub_seg);
  pub_seg->header.frame_id = "map";
  pub_seg->header.stamp = ros::Time::now();
  in_map_pub_.publish(*pub_seg);
}
void SignalHandler(int sig) { ros::shutdown(); }
int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_detection");
  LaserDetection laser_detection;
  signal(SIGTERM, SignalHandler);
  ros::spin();
  return 0;
}
