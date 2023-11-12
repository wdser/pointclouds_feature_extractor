#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <json/json.h>
#include <json/value.h>


class Feature {

  public:
  Eigen::Vector3f& GetCenter() { return center_; }
  pcl::Normal& GetNormal() { return feature_normal_; }

  private:

  Eigen::Vector3f center_;
  Eigen::Vector4f feature_coef_;
  pcl::Normal feature_normal_;
  std::vector<pcl::PointXYZI> feature_points_;
};

class FeatureExtractor {

  public:

  FeatureExtractor();
  FeatureExtractor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd_data,
                  const pcl::visualization::PCLVisualizer::Ptr viewer) 
                  : cloud_(pcd_data),viewer_(viewer) {}
  ~FeatureExtractor() = default;

  void SetKAndRadius(const int k_normal, const int k_feature, const double radius) {
    k_normal_ = k_normal;
    k_feature_ = k_feature;
    radius_ = radius;
  }
  void SetIsRadius(const bool is_radius) {
    is_radius_ = is_radius;
  }
  
  inline void SetInputData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd_data) {
    cloud_ = pcd_data;
  }

  Eigen::Vector3f PclToVector3f(pcl::PointXYZI point) {
    Eigen::Vector3f vec_point(point.x,
                                  point.y,
                                  point.z);
    return vec_point;
  }

  Eigen::Vector3f NormalToVector3f(pcl::Normal normal) {
    Eigen::Vector3f normal_vector(normal.normal_x,
                                  normal.normal_y,
                                  normal.normal_z);
    return normal_vector;
  }
  Eigen::Vector3f FeatureNormalToVector3f(Feature feature) {
    Eigen::Vector3f normal_vector(feature.GetNormal().normal_x,
                                    feature.GetNormal().normal_y,
                                    feature.GetNormal().normal_z);
    return normal_vector;
  }

  void ExtractFeatures();
  void RemoveNonFeaturePoints(const std::vector<int>& feature_points_index,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud);
  bool IsNormalCorplannar(pcl::Normal normal_a, pcl::Normal normal_b,
  pcl::PointXYZI search_point, pcl::PointXYZI searched_point);
  bool EstimateFeatureParameter(const std::vector<int>& feature_indexs,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                              Eigen::Vector4f* feature_coef);

  bool IsAngleCorplannar(Feature feature_a, Feature feature_b);
  bool IsDistanceCorplannar(Feature feature_a, Feature feature_b);
  void SVD(const Eigen::Matrix3Xf& points_3xf,
            Eigen::Matrix3f* singular_vectors, Eigen::Vector3f* singular_values);
  private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  pcl::KdTree<pcl::PointXYZI>::Ptr kdtree_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  // Json::Value config_;
  int k_normal_{0};
  int k_feature_{0};
  double radius_{0.0};
  bool is_radius_{true};
};