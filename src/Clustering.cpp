
#include "Clustering.h"
#include <cassert>
#include <tbb/parallel_for.h>

struct BucketSet {
    std::vector<char> bucket;
    size_t size;
    size_t now_first;
    void insert(size_t value)
    {
        if (!bucket[value]) {
            bucket[value] = true;
            ++size;
            if (value < now_first)
                now_first = value;
        }
    }
    void erase(size_t value)
    {
        if (bucket[value]) {
            bucket[value] = false;
            --size;
            if (size == 0)
                now_first = -1;
            else if (value == now_first) {
                for (size_t i = value + 1; i < bucket.size(); ++i)
                    if (bucket[i]) {
                        now_first = i;
                        break;
                    }
            }
        }
    }
    BucketSet(size_t real_size)
        : bucket(real_size, false)
        , size(0)
        , now_first(-1)
    {
    }
};

std::vector<int> DifferingDBSCAN(
    const open3d::geometry::PointCloud &cloud,
    const Eigen::Vector3d &zero_pos,
    double eps,
    double min_points_k)
{
    /// DBSCAN算法 根据距离二次反比聚类 min_points
    if (cloud.points_.empty()) {
        return std::vector<int>();
    }
    open3d::geometry::KDTreeFlann kdtree(cloud);

    std::vector<std::vector<int>> nbs(cloud.points_.size());
    // #pragma omp parallel for schedule(static) num_threads(open3d::utility::EstimateMaxThreads())
    // for (int idx = 0; idx < int(cloud.points_.size()); ++idx) {
    //     std::vector<double> dists2;
    //     kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
    // // #pragma omp critical
    // }
    tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud.points_.size())),
        [&](const tbb::blocked_range<int> &r) {
            for (int idx = r.begin(); idx < r.end(); ++idx) {
                std::vector<double> dists2;
                kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
            }
        });

    // 默认标签为 -2, 未被访问过`
    std::vector<int> labels(cloud.points_.size(), -2);
    int cluster_label = 0;
    for (size_t idx = 0; idx < cloud.points_.size(); ++idx) {
        // 标签已被确认, 跳过
        if (labels[idx] != -2) {
            continue;
        }
        // 令最少点数与距离的平方成反比
        size_t min_points = min_points_k / (cloud.points_[idx] - zero_pos).squaredNorm();
        // 如果邻域内点数不足, 标记为噪声
        if (nbs[idx].size() < min_points) {
            labels[idx] = -1;
            continue;
        }

        // 利用 unordered_set 去重
        BucketSet nbs_next(cloud.points_.size());
        BucketSet nbs_visited(cloud.points_.size());
        for (int nb : nbs[idx])
            nbs_next.insert(nb);
        nbs_visited.insert(int(idx));

        labels[idx] = cluster_label;
        // BFS 遍历邻域
        while (nbs_next.size > 0) {
            int nb = nbs_next.now_first;
            nbs_next.erase(nb);
            nbs_visited.insert(nb);

            // 噪声转为聚类中的点 (不用专门判断)
            // if (labels[nb] == -1)
            //     labels[nb] = cluster_label;
            // 在并行中, 有可能出现这种情况
            // if (labels[nb] != -2 && labels[nb] != -1)
            //     continue;
            // 不记入噪声点, 缩小聚类, 减少错误
            if (labels[nb] != -2)
                continue;

            labels[nb] = cluster_label;

            if (nbs[nb].size() >= min_points)
                for (int qnb : nbs[nb])
                    if (!nbs_visited.bucket[qnb])
                        nbs_next.insert(qnb);
        }

        cluster_label++;
    }

    return labels;
}

std::vector<int> NormalDBSCAN(
    const open3d::geometry::PointCloud& cloud,
    double eps,
    size_t min_points)
{
    open3d::geometry::KDTreeFlann kdtree(cloud);

    std::vector<std::vector<int>> nbs(cloud.points_.size());
    tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud.points_.size())),
        [&](const tbb::blocked_range<int> &r) {
            for (int idx = r.begin(); idx < r.end(); ++idx) {
                std::vector<double> dists2;
                kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
            }
        });

    std::vector<int> labels(cloud.points_.size(), -2);
    int cluster_label = 0;
    for (size_t idx = 0; idx < cloud.points_.size(); ++idx) {
        // Label is not undefined.
        if (labels[idx] != -2) {
            continue;
        }

        // Check density.
        if (nbs[idx].size() < min_points) {
            labels[idx] = -1;
            continue;
        }

        BucketSet nbs_next(cloud.points_.size());
        BucketSet nbs_visited(cloud.points_.size());
        for (int nb : nbs[idx])
            nbs_next.insert(nb);
        nbs_visited.insert(int(idx));

        labels[idx] = cluster_label;

        while (nbs_next.size > 0) {
            int nb = nbs_next.now_first;
            nbs_next.erase(nb);
            nbs_visited.insert(nb);

            // Noise label.
            if (labels[nb] == -1) {
                labels[nb] = cluster_label;
            }
            // Not undefined label.
            if (labels[nb] != -2) {
                continue;
            }
            labels[nb] = cluster_label;

            if (nbs[nb].size() >= min_points)
                for (int qnb : nbs[nb])
                    if (!nbs_visited.bucket[qnb])
                        nbs_next.insert(qnb);
        }

        cluster_label++;
    }

    return labels;
}
