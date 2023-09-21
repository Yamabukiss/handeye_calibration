#include "searcher.h"

Searcher::Searcher(double step, int max_timeout_count) :
    step_(step), timeout_count_(0), max_timeout_count_(max_timeout_count)
{

}

double Searcher::calculateErrorPerDimension(const Eigen::Vector4d &dimension_vec,
                                  const Eigen::MatrixXd &camera_mat)
{
    double error = (dimension_vec.transpose() * camera_mat).array().square().sum();
    return error;
}

void Searcher::backTrackLineSearch(
        const double &gradient, const double &error, int index,
        const Eigen::Vector4d& dimension_vec, const Eigen::MatrixXd &camera_mat,
        int try_times, double alpha, double beta)
{
    double tmp_step = step_;
    int count = 0;

    if (timeout_count_  >= max_timeout_count_)
        return;

    while (count < try_times)
    {
        auto tmp_vec = dimension_vec;
        double &arg = tmp_vec[index];
        arg -= tmp_step * gradient;
        double new_error = calculateErrorPerDimension(tmp_vec, camera_mat);

        if (new_error > error + tmp_step * alpha * gradient * -gradient)
            tmp_step *= beta;

        else
        {
            step_ = tmp_step;
            return;
        }

        count++;
    }
    timeout_count_++;
}

Searcher::~Searcher() = default;