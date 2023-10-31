#include "./plane_extractor.h"
#include <unordered_set>

// void PlaneExtractor::Planes () {

// }

// void PlaneExtractor::BuildKdTree () {
   
// }

bool PlaneExtractor::IsNormalCorplannar(pcl::Normal normal_a, pcl::Normal normal_b) {
  Eigen::Vector3f vector_a(normal_a.normal_x, normal_a.normal_y, normal_a.normal_z);
  Eigen::Vector3f vector_b(normal_b.normal_x, normal_b.normal_y, normal_b.normal_z);
  bool is_corplannar = std::abs(vector_a.norm() * vector_b.norm()) > 0.9 ? true :false;

  // double cosValNew = vector_a.dot(vector_b) /(vector_a.norm()*vector_b.norm());
  // double angle = acos(cosValNew) * 180 / M_PI;
  // bool is_corplannar = std::abs(angle) <  ? true : false;

  return is_corplannar;
}

void PlaneExtractor::ExtractPlanes () {
  // BuildKdTree
  // BuildKdTree();

  // kdtree_ = std::make_shared(new pcl::search::KdTree<pcl::PointXYZ>());
  // kdtree_->setInputCloud(cloud_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
      new pcl::search::KdTree<pcl::PointXYZ> ());

  tree->setInputCloud(cloud_);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimate;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  normal_estimate.setInputCloud(cloud_);
  normal_estimate.setSearchMethod(tree);
  normal_estimate.setKSearch(20);
  // normal_estimate.setRadiusSearch(0.25);
  
  std::cout << "normals size: " << normals->points.size() << std::endl;
  normal_estimate.compute(*normals);
  std::cout << "normals size: " << normals->points.size() << std::endl;

  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (
  // new pcl::PointCloud<pcl::PointNormal>);
  // pcl::concatenateFields (*cloud_, *normals, *cloud_with_normals);

  // pcl::visualization::PCLVisualizer viewer("Normals Viewer");
  // viewer.addPointCloud(cloud_, "cloud");
  // viewer.setPointCloudRenderingProperties(
  //   pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
  viewer_->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(
                    cloud_, normals,1, 0.05, "cloud_normals");

  int points_size = cloud_->points.size();
  std::queue<int> points_idx_queue;
  std::vector<int> plane_points_index(points_size,0);
  std::vector<int> curent_plane_points;
  std::vector<bool> is_point_visited(points_size,false);
  // plane_points_index.resize(points_size);
  int plane_idx = 0;

  for (int i = 0; i < points_size; ++i) {
    if (points_idx_queue.empty()) {
      if (curent_plane_points.size() > 10) {
        // found all points belong to currrent plane
        for (int j = 0; j < curent_plane_points.size(); j++) {
          int point_idx = curent_plane_points[j];
          plane_points_index[point_idx] = plane_idx;
        }
        std::cout << "plane: " << plane_idx << std::endl;
        std::cout << "curent_plane_points: " 
                  << curent_plane_points.size() << std::endl;
        ++plane_idx;
      }
      curent_plane_points.clear();
    }

    if (is_point_visited[i]) {
      continue;
    }

    if (points_idx_queue.empty()) {
      points_idx_queue.push(i);
      curent_plane_points.push_back(i);
      is_point_visited[i] = true;
    }

    while (!points_idx_queue.empty()) {
      int search_idx = points_idx_queue.front();
      points_idx_queue.pop();
      pcl::PointXYZ search_point = cloud_->points[search_idx];
      int K = 10;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      if(tree->nearestKSearch(search_point, K, 
          pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (size_t idx = 0; idx < K; ++idx) {
          int searched_idx = pointIdxNKNSearch[idx];
          if (is_point_visited[searched_idx]) {
            continue;
          }
          pcl::Normal search_normal = normals->points[search_idx];
          pcl::Normal searched_normal = normals->points[searched_idx];
          if (IsNormalCorplannar(search_normal,searched_normal)) {
            curent_plane_points.push_back(searched_idx);
            points_idx_queue.push(searched_idx);
            is_point_visited[searched_idx] = true;
          }
        }
      }
    }
  }
  int color = plane_idx ? 255 / plane_idx : 50;
  std::cout << "total planes: " << plane_idx << std::endl;


  // viewer.setPointCloudRenderingProperties(
  //   pcl::visualization::PCL_VISUALIZER_COLOR, 1, 10, 0, "normals");
}