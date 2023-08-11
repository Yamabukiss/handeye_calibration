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
    bool ret = cv::findChessboardCorners(image, pattern_size, corners_vec);

    refineCorner(image, corners_vec);
    cv::drawChessboardCorners(image, pattern_size, corners_vec, ret);
    int count = 1;
    for (const auto &point : corners_vec)
    {
        cv::Point point_i(static_cast<int>(point.x), static_cast<int>(point.y));
        cv::putText(image, std::to_string(count), point_i, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
        count++;
    }


    return {ret, corners_vec};
}

void ImageProc::refineCorner(const cv::Mat &image, std::vector<cv::Point2f> &corners_vec)
{
    auto tmp_image = image;
    cv::cvtColor(tmp_image, tmp_image, cv::COLOR_BGR2GRAY);
    cv::find4QuadCornerSubpix(tmp_image, corners_vec, cv::Size(5, 5));
}
