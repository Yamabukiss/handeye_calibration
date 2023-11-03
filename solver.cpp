#include "solver.h"
#include "searcher.h"

Solver::Solver(double precision, double init_trans_step) : init_trans_step_(init_trans_step)
{
    precision_ = pow(10, precision);
}

int Solver::searchDimension(
    const Eigen::Matrix4d &dimension_mat,
    const Eigen::MatrixXd &cam_points_mat,
    const Eigen::MatrixXd &base_points_mat,
    int num_points)
{
    auto tmp_mat = dimension_mat * cam_points_mat;
    auto estimate_cam_mat = tmp_mat.block(0, 0, 3, num_points);
    auto error_mat = (base_points_mat - estimate_cam_mat).array().square() / 2;
    auto tmp_vec = error_mat.rowwise().sum();
    auto error_sum_vec = tmp_vec.head(2);

    int dimension = 0;
    error_sum_vec.maxCoeff(&dimension);
    return dimension;
}

bool Solver::estimationOptimize(
        Eigen::Matrix4d &dimension_mat,
        const std::vector<Eigen::Vector4d>& cam_points_vec,
        const std::vector<Eigen::Vector4d>& base_points_vec,
        int iterations) {

    double result_cost = 0;
    size_t num_points = cam_points_vec.size();
    Eigen::MatrixXd cam_points_mat(4, static_cast<int>(num_points));
    Eigen::MatrixXd base_points_mat(3, static_cast<int>(num_points));

    for (size_t i = 0; i < num_points; i++)
        cam_points_mat.col(static_cast<int>(i)) = cam_points_vec[i];

    for (size_t i = 0; i < num_points; i++)
        base_points_mat.col(static_cast<int>(i)) = base_points_vec[i].head(3);

    int dimension = searchDimension(dimension_mat, cam_points_mat, base_points_mat, num_points);

    double cost = 0, last_cost = 0;

    Searcher w_searcher(init_trans_step_);

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        auto tmp_mat = dimension_mat * cam_points_mat;
        auto estimate_cam_mat = tmp_mat.block(0, 0, 3, num_points);
        auto error_mat = (base_points_mat - estimate_cam_mat).array().square() / 2;
        auto dimension_error = (base_points_mat - estimate_cam_mat).row(dimension);
        cost = error_mat.row(dimension).sum();

        result_cost = cost / num_points;

        double &w = dimension_mat(dimension, 3);

        double gradient_w = 0;

        for (int i = 0; i < num_points; i++)
            gradient_w += -dimension_error[i];

        w_searcher.backTrackLineSearch(gradient_w, cost, 3,
                                       dimension_mat.row(dimension), cam_points_mat);

        w -= gradient_w * w_searcher.step_;

        std::cout << "cost: " << cost << std::endl;
        std::cout << "last_cost: " << last_cost << std::endl;

        if ((iteration > 0 && (abs(std::round(cost * precision_)) >= abs(std::round(last_cost * precision_)))) ||
            abs(gradient_w * w_searcher.step_) < 0.01)
        {
            std::cout << "----------------" << std::endl;
            std::cout << "error: " << cost << std::endl;
            std::cout << "avg_error: " << result_cost << std::endl;
            std::cout << "dimension: " << dimension << std::endl;
            std::cout << "point1: " << (dimension_mat * cam_points_vec[0]) << std::endl;
            std::cout << "point2: " << (dimension_mat * cam_points_vec[1]) << std::endl;

            if (std::isnan(cost) || std::isinf(cost))
            {
                std::cout << dimension << std::endl;
                return false;
            }
            else
                break;
        }

        last_cost = cost;
    }
    std::cout << "------iteration_end----------" << std::endl;
    std::cout << "error: " << cost << std::endl;
    std::cout << "avg_error: " << result_cost << std::endl;
    std::cout << "dimension: " << dimension_mat.row(0) << std::endl;
    std::cout << "point1: " << (dimension_mat * cam_points_vec[0]) << std::endl;
    std::cout << "point2: " << (dimension_mat * cam_points_vec[1]) << std::endl;

    if (result_cost < 0.5)
        return true;
    else
        return false;
}

Eigen::Matrix4d Solver::svd(std::vector<cv::Point3d> _cam_points_vec, std::vector<cv::Point3d> _base_points_vec)
{
    Eigen::MatrixXd A(4, _cam_points_vec.size());
    Eigen::MatrixXd B(4, _base_points_vec.size());

    for (size_t i = 0; i < _cam_points_vec.size(); i++)
    {
        A.col(i) << _cam_points_vec[i].x, _cam_points_vec[i].y, _cam_points_vec[i].z, 1.0;
        B.col(i) << _base_points_vec[i].x, _base_points_vec[i].y, _base_points_vec[i].z, 1.0;
    }

    Eigen::Vector4d mA = A.rowwise().mean();
    Eigen::Vector4d mB = B.rowwise().mean();

    A = A.colwise() - mA;
    B = B.colwise() - mB;

    Eigen::Matrix4d H = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix4d U = svd.matrixU();
    Eigen::Matrix4d V = svd.matrixV();

    Eigen::Matrix4d R = V * U.transpose();

    double det_R = R.determinant();
    if (det_R < 0) {
        V.col(3) *= -1;
        R = V * U.transpose();
    }

    Eigen::Vector4d t = mB - R * mA;

    Eigen::Matrix4d transformation_mat = Eigen::Matrix4d::Identity();
    transformation_mat.block(0, 0, 3, 3) = R.block(0, 0, 3, 3);
    transformation_mat.block(0, 3, 3, 1) = t.block(0, 0, 3, 1);

    std::vector<Eigen::Vector4d> cam_points_vec, base_points_vec;
    cam_points_vec.reserve(A.rows());
    base_points_vec.reserve(B.rows());

    for (int row = 0; row < A.rows(); row++)
    {
        cam_points_vec.emplace_back(A.col(row));
        base_points_vec.emplace_back(B.col(row));
    }

    auto tmp_mat = transformation_mat;
    bool ret = estimationOptimize(tmp_mat, cam_points_vec, base_points_vec);

    if (ret)
        transformation_mat = tmp_mat;

    return transformation_mat;
}


Solver::~Solver()  = default;
