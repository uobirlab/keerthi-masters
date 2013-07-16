/*
	Author : Keerthi Kumar
	Note: Based on pcl tutorials
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

ros::Publisher pub; // publisher for publishing the segmented pcl.

// Pcl visulaiser to visualise the pcl. Use it if needed.
/*
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
*/

// Call back function for the subscriber to pcl data.
void process_cloud (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // to store the incoming pcl.
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>); // to store the segmented pcl
  sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2); // final segmented pcl in data type suitable for ros.

  // Convert to the templated PointCloud
  pcl::fromROSMsg (*input, *cloud);

  std::vector<int> inliers; // To store the points lying inside the plane model.

  // Perpendicular model plane
  pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud));

  model_p->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0)); // Set the axis to y-axis to detect horizontal planes.
  model_p->setEpsAngle (pcl::deg2rad (15.0)); // Set the angle in radians. Vary as per needs.
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p); // Use the RANSAC for segmentation and set the model plane chosen.
  ransac.setDistanceThreshold (.01);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // copies all inliers of the model computed to another PointCloud which should be published.
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  /* If using pcl visualiser :

  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(final);
  while (!viewer->wasStopped ())
  {
    	viewer->spinOnce (100);
   	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */


  // convert to sensormsg type
  pcl::toROSMsg (*final, *final_cloud);

 //std::cerr << "pcl cloud: " << cloud->width << " points" << std::endl;
 //std::cerr << "pcl final: " << final->width << " points" << std::endl;
 //std::cerr << "pcl final_cloud: " << final_cloud->width << " points" << std::endl;

 // Publish the segmented pcl.
 final_cloud->header.frame_id = input->header.frame_id;
 final_cloud->header.stamp = input->header.stamp;
 pub.publish (final_cloud);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tabletop");
  ros::NodeHandle nh; // Ros node handler.

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("depthcam", 1, process_cloud); // depthcam is the pcl2 received from kinect(actually from morse simulator)

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("new_pcl", 1000); // new pcl having only planar points and needs more buffer size.

  // Spin
  ros::spin ();
}
