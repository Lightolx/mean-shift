#include <iostream>
#include <Eigen/Eigen>
#include <fstream>

#include "cirlce.h"

int main()
{
    // Step0: Read in raw points
    std::ifstream fin("/media/psf/Home/Documents/MATLAB_codes/mean_shift/pt.txt");
    std::string ptline;
    double x, y;
    std::vector<Eigen::Vector2d> points;
    std::vector<double> Xs, Ys;

    while (getline(fin, ptline))
    {
        std::stringstream ss(ptline);
        ss >> x >> y;
        points.emplace_back(Eigen::Vector2d(x,y));
        Xs.push_back(x);
        Ys.push_back(y);
    }

    // Step1: Construct a kd_tree for neighbors searching circle and find its gravity center
    int ptNum = points.size();
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = ptNum;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->resize(cloud->width * cloud->height);

    for (int i = 0; i < cloud->size(); i++)
    {
        cloud->points[i].x = points[i](0);
        cloud->points[i].y = points[i](1);
    }

    pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
    kdTree->setInputCloud(cloud);

    Circle::cloud_ = cloud;
    Circle::kdTree_ = kdTree;

    // Step2: Construct a circle, start from a rand point to converge
    double Xmin = *(std::min_element(Xs.begin(), Xs.end()));
    double Xmax = *(std::max_element(Xs.begin(), Xs.end()));
    double Ymin = *(std::min_element(Ys.begin(), Ys.end()));
    double Ymax = *(std::max_element(Ys.begin(), Ys.end()));

    double radius = std::sqrt(pow(Xmax - Xmin, 2) + pow(Ymax - Ymin, 2)) / 8;
    Circle circle(points[0], radius);

    while(1)
    {
        circle.computeCenter();

        if (circle.offset() < radius/200)
        {
            break;
        }

        std::cout << "offset is " << circle.offset() << std::endl;
        circle.converge();
    }

    circle.computeCenter();

    // Step3: Write back 3D points and export it
    std::ofstream fout("/media/psf/Home/Documents/MATLAB_codes/mean_shift/pt1.txt");
    Eigen::Vector2d pt = circle.p_;
    fout << pt.x() << " " << pt.y() << " " << radius << std::endl;
    fout.close();

    fout.open("/media/psf/Home/Documents/MATLAB_codes/mean_shift/pt2.txt");
    for (const Eigen::Vector2d &pt : circle.neighbors_)
    {
        fout << pt.x() << " " << pt.y() << " " << std::endl;
    }
    fout.close();
}