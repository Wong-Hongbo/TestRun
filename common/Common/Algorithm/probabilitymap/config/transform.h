#ifndef TRANSFORM_H
#define TRANSFORM_H
#include <opencv2/opencv.hpp>
#include <vector>

namespace xs{
class LookUpTable{
public:
    LookUpTable();

    double cos_lookup_table[36001];
    double sin_lookup_table[36001];
};

class Transform{
public:
    Transform();
    ~Transform();

    bool isRotationMatrix(const cv::Mat &R);
    void eulerAnglesToRotationMatrix(const cv::Vec3i &theta, double R[3][3]);
    void eulerAnglesToRotationMatrix(const cv::Vec3d &theta, double R[3][3]);
    cv::Vec3f rotationMatrixToEulerAngels(const cv::Mat &R);

private:
    LookUpTable sincostable_;

};
}

#endif // TRANSFORM_H
