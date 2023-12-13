#ifndef IMAGEPROC_H
#define IMAGEPROC_H

#include <QObject>
#include <QPixmap>
#include <QImage>
#include <opencv2/opencv.hpp>

class ImageProc : public QObject
{
    Q_OBJECT
public:
    explicit ImageProc(QObject *parent = nullptr);

    std::pair<bool, std::vector<cv::Point2f>> findCorner(cv::Mat &image);

    cv::Mat pixmapToCvMat(const QPixmap& pixmap);

    void refineCorner(const cv::Mat &image, std::vector<cv::Point2f> &corners_vec);

    QPixmap matToPixmap(const cv::Mat& mat);

    std::vector<cv::Point> getCircle(cv::Mat image, int _gradient, int _area_thresh, int _struct_size);

signals:
};

#endif // IMAGEPROC_H
