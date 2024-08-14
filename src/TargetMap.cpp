#include "TargetMap.h"
#include "Config.h"
#include "Clustering.h"
#include "VoxelGrid.h"

using open3d::geometry::PointCloud;
using open3d::geometry::TriangleMesh;
using open3d::geometry::AxisAlignedBoundingBox;

open3d::visualization::ColorMapJet cm;
size_t flag_size, last_frames, combine_limit;

double loose_eps, loose_expand;
size_t loose_min_points;
double strict_eps;
size_t strict_min_points, seperate_limit;

double dis_thres, cc_thres, init_lost, combine_dist, combine_fdist, project_z;
Eigen::Matrix4d z_zip, z_zip_inv;


inline bool is_coincide(const open3d::geometry::AxisAlignedBoundingBox& aabb1,
    const open3d::geometry::AxisAlignedBoundingBox& aabb2)
{
    /// @brief 判断两个 AABB 是否重合
    // 已二维化
    return std::abs(aabb1.max_bound_(0) + aabb1.min_bound_(0) - aabb2.max_bound_(0) - aabb2.min_bound_(0)) <= aabb1.max_bound_(0) - aabb1.min_bound_(0) + aabb2.max_bound_(0) - aabb2.min_bound_(0) + cc_thres
        && std::abs(aabb1.max_bound_(1) + aabb1.min_bound_(1) - aabb2.max_bound_(1) - aabb2.min_bound_(1)) <= aabb1.max_bound_(1) - aabb1.min_bound_(1) + aabb2.max_bound_(1) - aabb2.min_bound_(1) + cc_thres;
}
inline bool is_contain(const open3d::geometry::AxisAlignedBoundingBox& aabb, Eigen::Vector3d pt)
{
    /// @brief 判断 AABB 是否包含点 pt
    // 已二维化
    return (pt.array() >= aabb.min_bound_.array() && pt.array() <= aabb.max_bound_.array())(Eigen::seq(0, 1)).all();
}
inline Eigen::Matrix3d vectorToOrthogonalMatrix(const Eigen::Vector3d& v)
{
    /// @brief 将向量转换为旋转矩阵
    // 将向量规范化
    Eigen::Vector3d axis = v.normalized();
    // 计算旋转角度
    double angle = acos(axis.dot(Eigen::Vector3d::UnitZ()));
    // 计算旋转轴
    Eigen::Vector3d rot_axis = -axis.cross(Eigen::Vector3d::UnitZ()).normalized();
    // 创建旋转矩阵
    Eigen::AngleAxisd rotation(angle, rot_axis);
    return rotation.toRotationMatrix();
}

void TargetMap::cluster(const PointCloud &pc, double eps_, size_t min_points_, std::vector<int>& labels, std::vector<std::shared_ptr<PointCloud> > &pcs, PointCloud &pc_noise,
    std::vector<AxisAlignedBoundingBox> &aabbs, std::vector<size_t> &pt_num, std::vector<Eigen::Vector3d> &grav)
{
    /// @brief DBSCAN 聚类
    if (pc.points_.empty())
        return;
    PointCloud pc_tmp = pc;
    pc_tmp.Transform(z_zip);
    labels = NormalDBSCAN(pc_tmp, eps_, min_points_);
    int max_l = *std::max_element(labels.begin(), labels.end());
    for (int i = 0; i <= max_l; i++)
        pcs.emplace_back(std::make_shared<PointCloud>());
    for (size_t i = 0; i < pc.points_.size(); i++)
        if (labels[i] >= 0)
            pcs[labels[i]]->points_.push_back(pc.points_[i]);
        else
            pc_noise.points_.push_back(pc.points_[i]);
    for (int i = 0; i <= max_l; i++) {
        aabbs.emplace_back(pcs[i]->GetAxisAlignedBoundingBox());
        pt_num.emplace_back(pcs[i]->points_.size());
        grav.emplace_back(pcs[i]->GetCenter());
    }
}

void TargetMap::cluster_d(const PointCloud &pc, const Eigen::Vector3d &zero_pos, double eps_, size_t min_points_k,
    std::vector<int>& labels, std::vector<std::shared_ptr<PointCloud> > &pcs, PointCloud &pc_noise,
    std::vector<AxisAlignedBoundingBox> &aabbs, std::vector<size_t> &pt_num, std::vector<Eigen::Vector3d> &grav)
{
    /// @brief DBSCAN 聚类 (带距离权重)
    if (pc.points_.empty())
        return;
    PointCloud pc_tmp = pc;
    pc_tmp.Transform(z_zip);
    labels = DifferingDBSCAN(pc_tmp, zero_pos, eps_, min_points_k);
    int max_l = *std::max_element(labels.begin(), labels.end());
    for (int i = 0; i <= max_l; i++)
        pcs.emplace_back(std::make_shared<PointCloud>());
    for (size_t i = 0; i < pc.points_.size(); i++)
        if (labels[i] >= 0)
            pcs[labels[i]]->points_.push_back(pc.points_[i]);
        else
            pc_noise.points_.push_back(pc.points_[i]);
    for (int i = 0; i <= max_l; i++) {
        aabbs.emplace_back(pcs[i]->GetAxisAlignedBoundingBox());
        pt_num.emplace_back(pcs[i]->points_.size());
        grav.emplace_back(pcs[i]->GetCenter());
    }
}

void TargetMap::initialize(const VoxelGrid &vg)
{
    /// @brief 初始化 TargetMap

    project_z = CFG2("TargetMap", "projectZ", double);

    project_func = [&](const Eigen::Vector3d& pt) -> Eigen::Vector3d {
        Eigen::Vector3d pt_grid = (pt - vg.bbox.min_bound_) / vg.voxel_size;
        Eigen::Vector3i pt_grid_int = pt_grid.cast<int>();
        return { pt(0), pt(1), vg.z_map(pt_grid_int(0), pt_grid_int(1)) + project_z };
    };

    flag_size = CFG2("TargetMap", "flagSize", size_t);
    last_frames = CFG2("TargetMap", "lastFrames", size_t);
    dis_thres = CFG2("TargetMap", "disThres", double);

    combine_limit = CFG2("TargetMap", "combineFrameLimit", size_t);
    combine_dist = CFG2("TargetMap", "combineDistance", double);
    combine_fdist = CFG2("TargetMap", "forceCombineDistance", double);
    loose_eps = CFG3("Cluster", "loose", "eps", double);
    loose_min_points = CFG3("Cluster", "loose", "minPoints", size_t);
    loose_expand = CFG3("Cluster", "loose", "boxExpand", double);

    seperate_limit = CFG2("TargetMap", "seperateFrameLimit", size_t);
    strict_eps = CFG3("Cluster", "strict", "eps", double);
    strict_min_points = CFG3("Cluster", "strict", "minPoints", size_t);

    cc_thres = CFG2("TargetMap", "ccThres", double);
    init_lost = CFG2("TargetMap", "initLost", size_t);

    double z_zip_c = CFG2("Cluster", "zZip", double);
    z_zip = Eigen::DiagonalMatrix<double, 4> { 1, 1, z_zip_c, 1 };
    z_zip_inv = Eigen::DiagonalMatrix<double, 4> { 1, 1, 1 / z_zip_c, 1 };

    target_map.resize(flag_size);

    inc_id = 0;
    for (size_t i = 0; i < flag_size; i++) {
        target_map[i].aabb = std::make_shared<AxisAlignedBoundingBox>();
        target_map[i].mesh = std::make_shared<TriangleMesh>();
        reset(i, true);
    }
}

void TargetMap::reset(size_t key, bool is_init)
{
    /// @brief 当一个目标完全不在跟踪队列中时, 重置该目标
    if (!is_init) {
        discarded_queue.push(target_map[key].id);
    }
    target_map[key].id = inc_id++;
    target_map[key].lost_time = init_lost;
    target_map[key].discarded = true;
    target_map[key].aabb->max_bound_ = { 0, 0, 0 };
    target_map[key].aabb->min_bound_ = { 0, 0, 0 };
    target_map[key].mesh->vertices_.clear();
    target_map[key].grav = { 0, 0, 0 };
    target_map[key].pt_num = 0;
    target_map[key].kf.init({ 0, 0, 0 }, init_lost);
}

void TargetMap::element_update(size_t key, const AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav)
{
    /// @brief 更新跟踪队列中的一个元素
    if (target_map[key].pt_num > 0) {
        target_map[key].aabb->max_bound_ = target_map[key].aabb->max_bound_.cwiseMax(aabb.max_bound_);
        target_map[key].aabb->min_bound_ = target_map[key].aabb->min_bound_.cwiseMin(aabb.min_bound_);
    }
    else {
        target_map[key].aabb->max_bound_ = aabb.max_bound_;
        target_map[key].aabb->min_bound_ = aabb.min_bound_;
    }
    // 更新重心, 对可能的多个聚类确定为同一目标的情况下, 进行平均
    target_map[key].grav = (pt_num * grav + target_map[key].pt_num * target_map[key].grav) / (pt_num + target_map[key].pt_num);
    target_map[key].pt_num += pt_num;
}

void TargetMap::pre_update()
{
    /// @brief 更新前阶段处理
    for (size_t key = 0; key < flag_size; key++) {
        if (target_map[key].discarded) continue;
        target_map[key].pt_num = 0;
    }
}

void TargetMap::post_update()
{
    /// @brief 更新后阶段处理, 包括更新卡尔曼滤波器, 更新可视化球体
    for (size_t key = 0; key < flag_size; key++) {
        if (target_map[key].discarded) {
            if (target_map[key].pt_num > 0) {
                target_map[key].kf.init(project_func(target_map[key].grav), target_map[key].lost_time);
                target_map[key].discarded = false;
            }
            else
                continue;
        }
        if (target_map[key].lost_time > last_frames) {
            reset(key);
            continue;
        }
        if (target_map[key].pt_num > 0) {
            target_map[key].kf.update(project_func(target_map[key].grav));
            target_map[key].lost_time = target_map[key].lost_time > 0 ? target_map[key].lost_time - 1 : 0;
        }
        else {
            target_map[key].kf.update();
            target_map[key].lost_time++;
        }
        Eigen::Vector3d v = target_map[key].kf.X(Eigen::seq(3, 5));
        auto arrow = TriangleMesh::CreateArrow(30, 60, 50 + 20 * v.norm(), 100);
        arrow->Rotate(vectorToOrthogonalMatrix(v), { 0, 0, 0 });
        *target_map[key].mesh = *TriangleMesh::CreateSphere(100);
        *target_map[key].mesh += *arrow;
        target_map[key].mesh->Translate(target_map[key].kf.pos());
        target_map[key].mesh->ComputeVertexNormals();
        target_map[key].mesh->PaintUniformColor(cm.GetColor(key / 10.0));
    }
}

void TargetMap::combine(size_t new_key, std::vector<size_t> old_keys)
{
    /// @brief 将多个跟踪目标合并为一个新的跟踪目标
    for (size_t key : old_keys) {
        // 如果跟丢时间不够久则不合并
        if (key == new_key || target_map[key].lost_time < combine_limit || target_map[key].discarded)
            continue;
        spdlog::info("TargetMap: Combine {} to {}", key, new_key);
        reset(key);
    }
}

void TargetMap::combine_force(size_t new_key, std::vector<size_t> old_keys)
{
    /// @brief 将多个跟踪目标合并为一个新的跟踪目标 (强制合并)
    for (size_t key : old_keys) {
        if (key == new_key || target_map[key].discarded)
            continue;
        spdlog::info("TargetMap: Force Combine {} to {}", key, new_key);
        reset(key);
    }
}

size_t TargetMap::push(const AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav, const PointCloud& pc, bool no_strict)
{
    /// @brief 尝试匹配已经存在的跟踪目标, 进行更新
    int64_t min_key = -1;
    double min_dis = -1;
    std::vector<size_t> combine_key_list, combine_force_key_list, seperate_key_list;
    for (size_t i = 0; i < flag_size; i++) {
        if (target_map[i].discarded) continue;
        double dis = (target_map[i].kf.pos() - grav).squaredNorm();
        if (is_contain(aabb, target_map[i].kf.pos()) && target_map[i].lost_time < seperate_limit)
            seperate_key_list.push_back(i);
        // 对于匹配目标距离过远的情况, 不进行更新
        if (dis > dis_thres * dis_thres && !is_coincide(aabb, *target_map[i].aabb))
            continue;
        if (dis <= combine_dist * combine_dist)
            combine_key_list.push_back(i);
        if (dis <= combine_fdist * combine_fdist)
            combine_force_key_list.push_back(i);
        if (dis < min_dis || min_key == -1) {
                min_key = i;
                min_dis = dis;
        }
    }
    // 如果 aabb 与多个目标重合, 则告知出现了错误合并, 需要进行分离
    if (seperate_key_list.size() > 1 && !no_strict)
    {
        spdlog::info("TargetMap: Seperate {} targets", seperate_key_list.size());
        seperate(aabb, pc);
        return TM_NEED_SEPERATE;
    }
    if (min_key != -1) {
        if (combine_key_list.size() > 1)
            combine(min_key, combine_key_list);
        if (combine_force_key_list.size() > 1)
            combine_force(min_key, combine_force_key_list);
        element_update(min_key, aabb, pt_num, grav);
        return min_key;
    }
    /// 没有匹配到目标, 进行插入
    return new_target(aabb, pt_num, grav);
}

size_t TargetMap::new_target(const AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav)
{
    /// @brief 插入新的跟踪目标
    for (size_t t = 0; t < flag_size; t++) {
        // 限制 pt_num 为 0 的目标, 从而倾向于将不同聚类识别为不同目标
        if (target_map[t].discarded && target_map[t].pt_num == 0) {
            element_update(t, aabb, pt_num, grav);
            spdlog::info("TargetMap: Added new target {}.", t);
            return t;
        }
    }
    spdlog::info("TargetMap: Out of FLAG_MAP_SIZE");
    return TM_FAILED_INSERT;
}

void TargetMap::loose_query(const PointCloud &pc)
{
    /// @brief 对于没有匹配到的目标, 进行松弛查询
    std::vector<AxisAlignedBoundingBox> aabbs;
    for (size_t i = 0; i < flag_size; i++) {
        if (!target_map[i].discarded && target_map[i].pt_num == 0 && target_map[i].lost_time < combine_limit)
            aabbs.push_back(*target_map[i].aabb);
    }
    for (const auto& aabb : aabbs) {
        AxisAlignedBoundingBox aabb_expand = aabb;
        aabb_expand.max_bound_.array() += loose_expand;
        aabb_expand.min_bound_.array() -= loose_expand;
        PointCloud pc_crop = *pc.Crop(aabb_expand);
        if (pc_crop.points_.empty()) continue;
        std::vector<std::shared_ptr<PointCloud> > pcs;
        PointCloud pc_noise_x;
        std::vector<int> labels;
        std::vector<open3d::geometry::AxisAlignedBoundingBox> aabbs_;
        std::vector<size_t> pt_num;
        std::vector<Eigen::Vector3d> grav;
        TargetMap::cluster(pc_crop, loose_eps, loose_min_points, labels, pcs, pc_noise_x, aabbs_, pt_num, grav);
        // spdlog::info("TargetMap: Loose query {} clusters.", pcs.size());
        for (size_t i = 0; i < aabbs_.size(); i++) {
            // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
            push(aabbs_[i], pt_num[i], grav[i], pc);
        }
    }
}

void TargetMap::seperate(const AxisAlignedBoundingBox &aabb, const PointCloud &pc)
{
    /// @brief 将一个跟踪目标分裂为多个新的跟踪目标
    PointCloud pc_cropped = *pc.Crop(aabb);

    std::vector<std::shared_ptr<PointCloud> > pcs;
    PointCloud pc_noise;
    std::vector<int> labels;
    std::vector<open3d::geometry::AxisAlignedBoundingBox> aabbs;
    std::vector<size_t> pt_num;
    std::vector<Eigen::Vector3d> grav;
    TargetMap::cluster(pc_cropped, strict_eps, strict_min_points, labels, pcs, pc_noise, aabbs, pt_num, grav);

    if (pcs.size() == 1) {
        spdlog::info("TargetMap: Seperate failed.");
        return;
    }

    for (size_t i = 0; i < aabbs.size(); i++) {
        // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
        push(aabbs[i], pt_num[i], grav[i], pc, true);
    }
}

std::vector<int> TargetMap::update(const std::vector<AxisAlignedBoundingBox>& aabbs,
    const std::vector<size_t>& pt_nums,
    const std::vector<Eigen::Vector3d>& grav,
    const PointCloud& pc)
{
    /// @brief 批次更新跟踪目标, 返回跟踪目标的id
    std::vector<int> ids;
    assert(aabbs.size() == pt_nums.size());
    assert(aabbs.size() == grav.size());
    pre_update();
    /// 贪心算法
    for (size_t i = 0; i < aabbs.size(); i++) {
        // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
        ids.push_back(push(aabbs[i], pt_nums[i], grav[i], pc));
    }
    // 对于没有匹配到的目标, 进行松弛查询
    loose_query(pc);
    post_update();
    return ids;
}
