#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>


class Plane {

  public:
  Eigen::Vector3f& GetCenter() { return center_; }
  pcl::Normal& GetNormal() { return plane_normal_; }

  private:

  Eigen::Vector3f center_;
  Eigen::Vector4f plane_coef_;
  pcl::Normal plane_normal_;
  std::vector<pcl::PointXYZI> plane_points_;
};

class PlaneExtractor {

  public:

  PlaneExtractor();
  PlaneExtractor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd_data,
                  const pcl::visualization::PCLVisualizer::Ptr viewer) 
                  : cloud_(pcd_data),viewer_(viewer) {}
  ~PlaneExtractor() = default;

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
  Eigen::Vector3f PlaneNormalToVector3f(Plane plane) {
    Eigen::Vector3f normal_vector(plane.GetNormal().normal_x,
                                    plane.GetNormal().normal_y,
                                    plane.GetNormal().normal_z);
    return normal_vector;
  }

  void ExtractPlanes();
  void RemoveNonPlanePoints(const std::vector<int>& plane_points_index,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud);
  bool IsNormalCorplannar(pcl::Normal normal_a, pcl::Normal normal_b,
  pcl::PointXYZI search_point, pcl::PointXYZI searched_point);
  bool EstimatePlaneParameter(const std::vector<int>& plane_indexs,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                              Eigen::Vector4f* plane_coef);

  bool IsAngleCorplannar(Plane plane_a, Plane plane_b);
  bool IsDistanceCorplannar(Plane plane_a, Plane plane_b);
  void SVD(const Eigen::Matrix3Xf& points_3xf,
            Eigen::Matrix3f* singular_vectors, Eigen::Vector3f* singular_values);
  private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  pcl::KdTree<pcl::PointXYZI>::Ptr kdtree_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};