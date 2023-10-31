#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>


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

  void ExtractPlanes();
  // void BuildKdTree();
  bool IsNormalCorplannar(pcl::Normal normal_a, pcl::Normal normal_b);
  private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};