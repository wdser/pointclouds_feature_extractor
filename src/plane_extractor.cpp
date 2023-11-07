#include "./plane_extractor.h"

#include <unordered_set>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
bool PlaneExtractor::IsNormalCorplannar(pcl::Normal search_normal, pcl::Normal searched_normal,
pcl::PointXYZI search_point, pcl::PointXYZI searched_point) {
  Eigen::Vector3f normal_vector_a = NormalToVector3f(search_normal);
  Eigen::Vector3f normal_vector_b = NormalToVector3f(searched_normal);
  Eigen::Vector3f search_vec_point = PclToVector3f(search_point);
  Eigen::Vector3f searched_vec_point = PclToVector3f(searched_point);
  const double a_norm = normal_vector_a.norm();
  const double b_norm = normal_vector_b.norm();
  if (normal_vector_a.dot(normal_vector_b) /a_norm * b_norm > 0.9 
      && std::abs(normal_vector_a.dot(search_vec_point) - normal_vector_a.dot(searched_vec_point)) < 0.1) {
    return true;
  }
  return false;
}

bool PlaneExtractor::IsAngleCorplannar(Plane plane_a, Plane plane_b) {
  Eigen::Vector3f normal_vector_a = PlaneNormalToVector3f(plane_a);
  Eigen::Vector3f normal_vector_b = PlaneNormalToVector3f(plane_b);
  bool  is_angle_corplannar = std::abs(normal_vector_a.cross(normal_vector_b).norm())
                            > 0.9 ? true: false;
  return is_angle_corplannar;
}

bool PlaneExtractor::IsDistanceCorplannar(Plane plane_a, Plane plane_b) {
  Eigen::Vector3f normal_vector_a = PlaneNormalToVector3f(plane_a);
  Eigen::Vector3f normal_vector_b = PlaneNormalToVector3f(plane_b);
  Eigen::Vector3f vector_ab = normal_vector_a - normal_vector_b;
  bool  is_dis_corplannar = std::abs(vector_ab.dot(normal_vector_a)) 
                          + std::abs(vector_ab.dot(normal_vector_b))
                          < 0.2 ? true: false;
  return is_dis_corplannar;
}


bool PlaneExtractor::EstimatePlaneParameter(const std::vector<int>& plane_indexs,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                              Eigen::Vector4f* plane_coef) {
  Eigen::Matrix3Xf points_3xf(3,plane_indexs.size());
  for (int index = 0; index < plane_indexs.size(); index++) {
    pcl::PointXYZI point = point_cloud->points[index];
    points_3xf.col(index) = Eigen::Vector3f(point.x, point.y, point.z);
  }
  Eigen::Matrix3f singular_vectors;
  Eigen::Vector3f singular_values;
  SVD(points_3xf,&singular_vectors,&singular_values);
  return std::fabs(singular_values(0)) >= 1e-10 &&
          std::fabs(singular_values(2) / singular_values(0)) <
              0.1 &&
          std::fabs(singular_values(2)) <
              std::fabs(singular_values(1)) - 1e-10;
}

void  PlaneExtractor::SVD(const Eigen::Matrix3Xf& points_3xf,Eigen::Matrix3f* singular_vectors, Eigen::Vector3f* singular_values) {
  Eigen::Vector3f center_point = points_3xf.rowwise().mean();
  Eigen::Matrix3Xf eigen_matrix = points_3xf.colwise() - center_point;
  cv::Mat cv_matrix;
  cv::eigen2cv(eigen_matrix, cv_matrix);
  cv::SVD cv_svd(cv_matrix);
  cv::cv2eigen(cv_svd.w, *singular_values);
  cv::cv2eigen(cv_svd.u, *singular_vectors);
  
  if (center_point.dot(singular_vectors->rightCols<1>()) > 0) {
    singular_vectors->col(0) = -singular_vectors->col(0);
    singular_vectors->col(2) = -singular_vectors->col(2);
  }
  Eigen::Vector3f cross_vec =
      singular_vectors->col(0).cross(singular_vectors->col(1));
  if (cross_vec.dot(singular_vectors->col(2)) > 0) {
    singular_vectors->col(1) = -singular_vectors->col(1);
  }
}

void PlaneExtractor::ExtractPlanes () {

  // kdtree_ = std::make_shared(new pcl::search::KdTree<pcl::PointXYZI>());
  // kdtree_->setInputCloud(cloud_);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (
      new pcl::search::KdTree<pcl::PointXYZI> ());
  tree->setInputCloud(cloud_);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimate;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  normal_estimate.setInputCloud(cloud_);
  normal_estimate.setSearchMethod(tree);
  // normal_estimate.setKSearch(20);
  normal_estimate.setRadiusSearch(0.25);
  
  std::cout << "normals size: " << normals->points.size() << std::endl;
  normal_estimate.compute(*normals);
  std::cout << "normals size: " << normals->points.size() << std::endl;

  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (
  // new pcl::PointCloud<pcl::PointNormal>);
  // pcl::concatenateFields (*cloud_, *normals, *cloud_with_normals);

  // pcl::visualization::PCLVisualizer viewer("Normals Viewer");
  // viewer.addPointCloud<pcl::PointXYZI>(cloud_, "cloud");
  // viewer.setPointCloudRenderingProperties(
  //   pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
  viewer_->addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(
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
        Eigen::Vector4f plane_coef;
        if (!EstimatePlaneParameter(curent_plane_points,cloud_,&plane_coef)) {
          curent_plane_points.clear();
          continue;
        }
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
      pcl::PointXYZI search_point = cloud_->points[search_idx];

      // int K = 50;
      // std::vector<int> pointIdxNKNSearch(K);
      // std::vector<float> pointNKNSquaredDistance(K);
      // if(tree->nearestKSearch(search_point, K, 
      //     pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      //   for (size_t idx = 0; idx < K; ++idx) {
      //     int searched_idx = pointIdxNKNSearch[idx];
      //     if (is_point_visited[searched_idx]) {
      //       continue;
      //     }

      const double radius = 0.5;
      pcl::	Indices k_indices;
      std::vector<float> k_sqr_distances;
      if(tree->radiusSearch(search_point, radius, 
          k_indices, k_sqr_distances) > 0) {
        for (size_t idx = 0; idx < k_indices.size(); ++idx) {
          int searched_idx = k_indices[idx];
          if (is_point_visited[searched_idx]) {
            continue;
          }

          pcl::Normal search_normal = normals->points[search_idx];
          pcl::Normal searched_normal = normals->points[searched_idx];
          pcl::PointXYZI searched_point = cloud_->points[searched_idx];
          if (IsNormalCorplannar(search_normal,searched_normal,search_point,searched_point)) {
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

  std::cout << " cloud_ size: " << cloud_->points.size() << std::endl;
  RemoveNonPlanePoints(plane_points_index,&cloud_);
  std::cout << " cloud_ size: " << cloud_->points.size() << std::endl;


  viewer_->addPointCloud<pcl::PointXYZI>(cloud_, "plane_points");
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "plane_points");

  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> single_color(cloud_, "intensity");
  // viewer_->addPointCloud<pcl::PointXYZI>(cloud_, single_color, "plane_points");

  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane_points");
  // viewer.setPointCloudRenderingProperties(
  //   pcl::visualization::PCL_VISUALIZER_COLOR, 1, 10, 0, "normals");
}

void PlaneExtractor::RemoveNonPlanePoints(const std::vector<int>& plane_points_index,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud) {
  pcl::PointIndices idx;
  for (int i = 0; i < plane_points_index.size(); ++i) {
    (*cloud)->points[i].intensity = plane_points_index[i];
    if (plane_points_index[i] > 0) {
      idx.indices.push_back(i);
    }
  }
  pcl::copyPointCloud(*(*cloud), idx.indices, *(*cloud));
  // (*cloud)
  return;
}