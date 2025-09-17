//
// Created by xiang on 2022/3/15.
//

#include "ch6/icp_2d.h"
#include "common/math_utils.h"

#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>

namespace sad {

// 实现了基于高斯牛顿法的点到点2D ICP
bool Icp2d::AlignGaussNewton(SE2& init_pose) {
    int iterations = 10;//高斯牛顿迭代次数
    double cost = 0, lastCost = 0;
    SE2 current_pose = init_pose;//初始的位置（机器人的pose）
    const float max_dis2 = 0.01;    // 最近邻时的最远距离（平方）--->最邻近点的最大平方距离
    const int min_effect_pts = 20;  // 最少的有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数

        // 遍历source的每个点
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];//获取当前激光束的距离（极坐标系下）
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;//当前激光束的角度=初始角度+第i个激光点*角度增益
            float theta = current_pose.so2().log();//此外当前机器人的航向角（so2应该只是选择其旋转部分，log即从李群转为李代数）
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));//获取点在body frame下的位置
            Point2d pt;//Vec2d--->Point2d格式的转换
            pt.x = pw.x();
            pt.y = pw.y();

            // 最近邻
            std::vector<int> nn_idx;//搜到的最邻近点的索引
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dis);//kd tree寻找最邻近，用的应该也是pcl的库。注意此处自搜了最邻近的一个点
            
            if (nn_idx.size() > 0 && dis[0] < max_dis2) {//对于搜到的最邻近点的索引，并且最近邻时的最远距离少于阈值
                effective_num++;
                Mat32d J;
                J << 1, 0, 0, 1, -r * std::sin(angle + theta), r * std::cos(angle + theta);//对应第六章的公式6.9，计算雅可比矩阵。
                H += J * J.transpose();//计算H矩阵

                Vec2d e(pt.x - target_cloud_->points[nn_idx[0]].x, pt.y - target_cloud_->points[nn_idx[0]].y);//计算两个点之间的误差
                b += -J * e;//计算b矩阵

                cost += e.dot(e);//误差的平方就是cost（欧氏距离）
            }
        }

        if (effective_num < min_effect_pts) {//有效的点不能少于这个数，所谓的有效点其实就是：少于阈值的最近邻时点数
            return false;
        }

        // solve for dx，奇异值分解求解最下二乘法
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {//求解有nan值，就是失效
            break;
        }

        cost /= effective_num;//求出平均的loss
        if (iter > 0 && cost >= lastCost) {//如果当前的cost比lastCost更大，那么就是上一个收敛了，就break
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        // 高斯牛顿迭代。x=x+dx
        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);//（so2应该只是选择其旋转部分，exp即从李代数转为李群）
        lastCost = cost;
    }

    // 迭代完后，
    init_pose = current_pose;//将位置更新
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}

// 点到面的ICP（2D情况下算是点到线的ICP）
bool Icp2d::AlignGaussNewtonPoint2Plane(SE2& init_pose) {
    int iterations = 10;
    double cost = 0, lastCost = 0;
    SE2 current_pose = init_pose;
    const float max_dis = 0.3;      // 最近邻时的最远距离
    const int min_effect_pts = 20;  // 最小有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数

        // 遍历source
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 查找5个最近邻（点到点查找的为1个最邻近，此处为5个）
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 5, nn_idx, dis);

            std::vector<Vec2d> effective_pts;  // 有效点（5个最邻近点中的有效点）
            for (int j = 0; j < nn_idx.size(); ++j) {
                if (dis[j] < max_dis) {
                    effective_pts.emplace_back(
                        Vec2d(target_cloud_->points[nn_idx[j]].x, target_cloud_->points[nn_idx[j]].y));
                }
            }

            if (effective_pts.size() < 3) {//弱有效点少于3个，则不执行
                continue;
            }

            // 拟合直线，组装J、H和误差
            Vec3d line_coeffs;
            if (math::FitLine2D(effective_pts, line_coeffs)) {//将搜到的k个邻近点拟合成直线
                effective_num++;
                Vec3d J;//雅可比矩阵的求解请见第六章的公式6.17
                J << line_coeffs[0], line_coeffs[1],
                    -line_coeffs[0] * r * std::sin(angle + theta) + line_coeffs[1] * r * std::cos(angle + theta);
                H += J * J.transpose();

                double e = line_coeffs[0] * pw[0] + line_coeffs[1] * pw[1] + line_coeffs[2];
                b += -J * e;

                cost += e * e;
            }
        }

        if (effective_num < min_effect_pts) {
            return false;
        }

        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        if (iter > 0 && cost >= lastCost) {
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        lastCost = cost;
    }

    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}

//对于输入的点云构建kdTree
void Icp2d::BuildTargetKdTree() {
    if (target_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }

    target_cloud_.reset(new Cloud2d);
    for (size_t i = 0; i < target_scan_->ranges.size(); ++i) {//对于每个点
        if (target_scan_->ranges[i] < target_scan_->range_min || target_scan_->ranges[i] > target_scan_->range_max) {
            continue;
        }

        double real_angle = target_scan_->angle_min + i * target_scan_->angle_increment;//获得角度信息

        Point2d p;//获得点云的xy信息
        p.x = target_scan_->ranges[i] * std::cos(real_angle);
        p.y = target_scan_->ranges[i] * std::sin(real_angle);
        target_cloud_->points.push_back(p);
    }

    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;
    kdtree_.setInputCloud(target_cloud_);//此处应该用的是PCL的库来构建kdTree
}

}  // namespace sad