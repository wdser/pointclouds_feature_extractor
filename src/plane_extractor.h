#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class PlaneExtractor {

  public:

  PlaneExtractor();
  PlaneExtractor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_data) : pcd_data_(pcd_data) {}
  ~PlaneExtractor() = default;

  inline void SetInputData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_data) {
    pcd_data_ = pcd_data;
  }

  void ExtractPlanes();
  void BuildKdTree();

  private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_data_;
};