#include "imageproc.h"

ImageProc::ImageProc(QObject *parent)
    : QObject{parent}
{

}

cv::Mat ImageProc::pixmapToCvMat(const QPixmap& pixmap)
{
    QImage image = pixmap.toImage();
    cv::Mat mat(image.height(), image.width(), CV_8UC4, image.bits(), image.bytesPerLine());
    cv::cvtColor(mat, mat, cv::COLOR_BGRA2BGR); // 将BGRA格式转换为BGR
    return mat;
}

QPixmap ImageProc::matToPixmap(const cv::Mat& mat)
{
    if (mat.empty()) {
        return QPixmap();
    }

    cv::Mat rgbMat;
    cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);

    QImage image(rgbMat.data, rgbMat.cols, rgbMat.rows, static_cast<int>(rgbMat.step), QImage::Format_RGB888);

    return QPixmap::fromImage(image);
}

std::pair<bool, std::vector<cv::Point2f>> ImageProc::findCorner(cv::Mat &image)
{
    cv::Size pattern_size(8, 11);
    std::vector<cv::Point2f> corners_vec;
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    bool ret = cv::findChessboardCorners(image, pattern_size, corners_vec);
    if (ret)
    {
        refineCorner(image, corners_vec);
        cv::drawChessboardCorners(image, pattern_size, corners_vec, ret);
        int count = 1;
        for (const auto &point : corners_vec)
        {
            cv::Point point_i(static_cast<int>(point.x), static_cast<int>(point.y));
            cv::putText(image, std::to_string(count), point_i, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            count++;
        }
    }

    return {ret, corners_vec};
}

void ImageProc::refineCorner(const cv::Mat &image, std::vector<cv::Point2f> &corners_vec)
{
    auto tmp_image = image;
    cv::find4QuadCornerSubpix(tmp_image, corners_vec, cv::Size(5, 5));
}

std::vector<cv::Vec3d> ImageProc::getCircle(cv::Mat image, int _dp, int _minDist, int _param1,
                          int _param2, int _minRadius, int _maxRadius)
{
    std::vector<cv::Vec3f> circles;
    cv::Mat gray = cv::Mat::zeros(image.rows, image.cols, CV_8U);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, _dp, _minDist, _param1, _param2, _minRadius, _maxRadius);

    std::vector<cv::Vec3d> tmp_circles;

    for (const cv::Vec3f& circle : circles)
    {
        cv::Vec3d tmp_circle(circle[0], circle[1], circle[2]);
        tmp_circles.push_back(tmp_circle);
    }

    return tmp_circles;
}
