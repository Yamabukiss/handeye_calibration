#ifndef IMAGEPROC_H
#define IMAGEPROC_H

#include <QObject>
#include <QPixmap>
#include <QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageProc : public QObject
{
    Q_OBJECT
public:
    explicit ImageProc(QObject *parent = nullptr);

    std::pair<bool, std::vector<cv::Point2f>> findCorner(cv::Mat &image);

    cv::Mat pixmapToCvMat(const QPixmap& pixmap);

    void refineCorner(const cv::Mat &image, std::vector<cv::Point2f> &corners_vec);

    QPixmap matToPixmap(const cv::Mat& mat);

signals:
};

#endif // IMAGEPROC_H
