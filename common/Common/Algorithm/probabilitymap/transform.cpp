#include "config/transform.h"
#include "config/util.h"
namespace xs{
LookUpTable::LookUpTable()
{
    for (int i = 0; i <= 36001; i++)
    {
        double rad = DegToRad(i/100.0f);
        sin_lookup_table[i] = sin(rad);
        cos_lookup_table[i] = cos(rad);
    }
}

Transform::Transform()
{

}

Transform::~Transform()
{

}

// Checks if a matrix is a valid rotation matrix.
bool Transform::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat should_be_identity = Rt*R;
    cv::Mat I = cv::Mat::eye(3, 3, should_be_identity.type());
    return cv::norm(I, should_be_identity) < 1e-6;
}

void Transform::eulerAnglesToRotationMatrix(const cv::Vec3i &theta, double R[3][3])
{
    int roll, pitch, yaw;
    roll = theta[0];
    pitch = theta[1];
    yaw = theta[2];
    roll = NormalizeDegreeTo36000(roll);
    pitch = NormalizeDegreeTo36000(pitch);
    yaw = NormalizeDegreeTo36000(yaw);
    double cr, cp, cy;
    double sr, sp, sy;
    cr = sincostable_.cos_lookup_table[roll];
    cp = sincostable_.cos_lookup_table[pitch];
    cy = sincostable_.cos_lookup_table[yaw];

    sr = sincostable_.sin_lookup_table[roll];
    sp = sincostable_.sin_lookup_table[pitch];
    sy = sincostable_.sin_lookup_table[yaw];

    R[0][0] = cy*cp;
    R[0][1] = -sy*cr + cy*sr*sp;
    R[0][2] = sy*sr + cy*cr*sp;
    R[1][0] = sy*cp;
    R[1][1] = cy*cr + sy*sr*sp;
    R[1][2] = -cy*sr + sy*cr*sp;
    R[2][0] = -sp;
    R[2][1] = cp*sr;
    R[2][2] = cp*cr;
//    cv::Mat R = (cv::Mat_<double>(3,3) <<
//                 cy*cp,  -sy*cr + cy*sr*sp,   sy*sr + cy*cr*sp,
//                 sy*cp,   cy*cr + sy*sr*sp,  -cy*sr + sy*cr*sp,
//                 -sp,     cp*sr,              cp*cr
//                 );
//    std::cout << cy*cp <<" " <<-sy*cr + cy*sr*sp <<" " <<  sy*sr + cy*cr*sp<<"\n" <<
//            sy*cp<<" " <<cy*cr + sy*sr*sp<<" " <<-cy*sr + sy*cr*sp<<"\n" <<
//            -sp<<" " <<    cp*sr<<" " <<             cp*cr << std::endl;
//    // Calculate rotation about x axis
//    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
//            1,       0,              0,
//            0,       cr,   -cp,
//            0,       cp,   cr
//            );

//    // Calculate rotation about y axis
//    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
//            cp,    0,      sp,
//            0,             1,      0,
//            -sp,   0,      cp
//            );

//    // Calculate rotation about z axis
//    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
//            cy,    -sy,      0,
//            sy,    cy,       0,
//            0,             0,                1
//            );


//    // Combined rotation matrix
//    cv::Mat R = R_z * R_y * R_x;

}


void Transform::eulerAnglesToRotationMatrix(const cv::Vec3d &theta, double R[3][3])
{
    double roll, pitch, yaw;
    roll = theta[0];
    pitch = theta[1];
    yaw = theta[2];

    double cr, cp, cy;
    double sr, sp, sy;
    cr = std::cos(roll);
    cp = std::cos(pitch);
    cy = std::cos(yaw);

    sr = std::sin(roll);
    sp = std::sin(pitch);
    sy = std::sin(yaw);

    R[0][0] = cy*cp;
    R[0][1] = -sy*cr + cy*sr*sp;
    R[0][2] = sy*sr + cy*cr*sp;
    R[1][0] = sy*cp;
    R[1][1] = cy*cr + sy*sr*sp;
    R[1][2] = -cy*sr + sy*cr*sp;
    R[2][0] = -sp;
    R[2][1] = cp*sr;
    R[2][2] = cp*cr;
//    cv::Mat R = (cv::Mat_<double>(3,3) <<
//                 cy*cp,  -sy*cr + cy*sr*sp,   sy*sr + cy*cr*sp,
//                 sy*cp,   cy*cr + sy*sr*sp,  -cy*sr + sy*cr*sp,
//                 -sp,     cp*sr,              cp*cr
//                 );
//    std::cout << cy*cp <<" " <<-sy*cr + cy*sr*sp <<" " <<  sy*sr + cy*cr*sp<<"\n" <<
//            sy*cp<<" " <<cy*cr + sy*sr*sp<<" " <<-cy*sr + sy*cr*sp<<"\n" <<
//            -sp<<" " <<    cp*sr<<" " <<             cp*cr << std::endl;
//    // Calculate rotation about x axis
//    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
//            1,       0,              0,
//            0,       cr,   -cp,
//            0,       cp,   cr
//            );

//    // Calculate rotation about y axis
//    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
//            cp,    0,      sp,
//            0,             1,      0,
//            -sp,   0,      cp
//            );

//    // Calculate rotation about z axis
//    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
//            cy,    -sy,      0,
//            sy,    cy,       0,
//            0,             0,                1
//            );


//    // Combined rotation matrix
//    cv::Mat R = R_z * R_y * R_x;

}


cv::Vec3f Transform::rotationMatrixToEulerAngels(const cv::Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}
}
