//
// Created by lightol on 4/18/18.
//

#ifndef MEAN_SHIFT_CIRLCE_H
#define MEAN_SHIFT_CIRLCE_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXY PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Circle
{
public:
    Circle() {}
    Circle(const Eigen::Vector2d &pt, double radius): p_(pt), c_(0.0, 0.0), radius_(radius) {}
    Eigen::Vector2d p_;
    std::vector<Eigen::Vector2d> neighbors_;
    void computeCenter();
    void converge() { p_ = c_; }
    double offset() { return (p_ - c_).norm(); }

    static pcl::search::KdTree<PointT>::Ptr kdTree_;
    static PointCloudT::Ptr cloud_;

private:
    // Get the Gaussian function value of x in sigma
    double gaussian(double x, double sigma = 3)
    {
        return exp(-0.5 * pow(x / sigma, 2)) / (pow(2 * M_PI, 0.5) * sigma);
    }

private:
    Eigen::Vector2d c_;
    double radius_;
};

#endif //MEAN_SHIFT_CIRLCE_H
