//
// Created by lightol on 4/19/18.
//

#include "cirlce.h"


PointCloudT::Ptr Circle::cloud_ = PointCloudT::Ptr();
pcl::search::KdTree<PointT>::Ptr Circle::kdTree_ = pcl::search::KdTree<PointT>::Ptr();

void Circle::computeCenter()
{
    PointT sPoint;
    sPoint.x = p_.x();
    sPoint.y = p_.y();

    std::vector<int> tmpPtsID;
    std::vector<float> tmpPtsSquaredDist;
    Eigen::Vector2d neighbor(0, 0);

    int neighborNum = kdTree_->radiusSearch(sPoint, radius_, tmpPtsID, tmpPtsSquaredDist);
    if (neighborNum > 1)
    {
        neighbors_.clear();
        neighbors_.reserve(neighborNum);

        int neiborNum = tmpPtsID.size();
        int id = 0;

        for (int i = 1; i < neiborNum; ++i)
        {
            id = tmpPtsID[i];
            neighbor.x() = cloud_->points[id].x;
            neighbor.y() = cloud_->points[id].y;
            neighbors_.push_back(neighbor);
        }
    }

    c_ = Eigen::Vector2d(0.0, 0.0);
    double weights(0.0), weight(0.0);
    volatile int num = neighbors_.size();
    for (const Eigen::Vector2d &pt : neighbors_)
    {
        weight = gaussian((pt - p_).norm(), radius_/3);
        weights += weight;
        c_ += weight * pt;
//        c_ += pt;
    }
    c_ /= weights;
}

