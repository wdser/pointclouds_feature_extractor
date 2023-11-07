#include <chrono>
#include <thread>
#include <cmath>

#include <json/json.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/gpu/octree/octree.hpp>

#include "./plane_extractor.h"
int main(int argc, char **argv)
{
  Json::Reader reader;
  Json::Value root;
  std::string data_path;
  std::ifstream in("../config/plane_extractor_config.json", std::ios::binary);

  if (reader.parse(in, root))
  {
    data_path = root["config_path"].asString();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_data(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(data_path, *pcd_data);
  std::cout << "size: " << pcd_data->points.size() << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
  viewer->addCoordinateSystem(3);
  viewer->addPointCloud<pcl::PointXYZI>(pcd_data, "pcd_data");
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "pcd_data");
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcd_data");

  // build kd tree
  PlaneExtractor plane_extractor(pcd_data,viewer);
  plane_extractor.ExtractPlanes();

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    // viewer->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}

