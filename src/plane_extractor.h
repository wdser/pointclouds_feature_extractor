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
  pcl::Normal plane_normal_;
  std::vector<pcl::PointXYZ> plane_points_;
};

class PlaneExtractor {

  public:

  PlaneExtractor();
  PlaneExtractor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_data,
                  const pcl::visualization::PCLVisualizer::Ptr viewer) 
                  : cloud_(pcd_data),viewer_(viewer) {}
  ~PlaneExtractor() = default;

  inline void SetInputData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_data) {
    cloud_ = pcd_data;
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
  // void BuildKdTree();
  bool IsNormalCorplannar(pcl::Normal normal_a, pcl::Normal normal_b);

  bool IsAngleCorplannar(Plane plane_a, Plane plane_b);
  bool IsDistanceCorplannar(Plane plane_a, Plane plane_b);
  private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};