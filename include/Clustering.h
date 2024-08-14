
#include <vector>
#include <deque>
#include <open3d/Open3D.h>


std::vector<int> DifferingDBSCAN(
    const open3d::geometry::PointCloud& cloud_ptr,
    const Eigen::Vector3d& zero_pos,
    double eps,
    double min_points_k);

std::vector<int> NormalDBSCAN(
    const open3d::geometry::PointCloud& cloud_ptr,
    double eps,
    size_t min_points);
