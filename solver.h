#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


class Solver
{
public:
    explicit Solver(double precision, double init_trans_step = 0.1);

    ~Solver();

    Eigen::Matrix4d svd(std::vector<cv::Point3d> _cam_points_vec, std::vector<cv::Point3d> _base_points_vec);

private:
    bool estimationOptimize(
        Eigen::Matrix4d &dimension_mat,
        const std::vector<Eigen::Vector4d>& cam_points_vec,
        const std::vector<Eigen::Vector4d>& base_points_vec,
        int iterations = 500);

    int searchDimension(
        const Eigen::Matrix4d &dimension_mat,
        const Eigen::MatrixXd &cam_points_mat,
        const Eigen::MatrixXd &base_points_mat,
        int num_points);

    double precision_;
    double init_trans_step_;
};


