#include "transform.h"

Transform::Transform() {}

Transform::~Transform() {}

// Checks if a matrix is a valid rotation matrix.
bool Transform::IsRotationMatrix(const cv::Mat &R) {
  cv::Mat Rt;
  cv::transpose(R, Rt);
  cv::Mat should_be_identity = Rt * R;
  cv::Mat I = cv::Mat::eye(3, 3, should_be_identity.type());
  return cv::norm(I, should_be_identity) < 1e-6;
}

void Transform::EulerAnglesToRotationMatrix(const cv::Vec3i &theta,
                                            double R[3][3]) {
  int roll, pitch, yaw;
  roll = theta[0];
  pitch = theta[1];
  yaw = theta[2];
  roll = NormalizeDegreeTo36000(roll);
  pitch = NormalizeDegreeTo36000(pitch);
  yaw = NormalizeDegreeTo36000(yaw);
  double cr, cp, cy;
  double sr, sp, sy;
  cr = SinuLookupTable::Instance()->CosValue(roll);
  cp = SinuLookupTable::Instance()->CosValue(pitch);
  cy = SinuLookupTable::Instance()->CosValue(yaw);

  sr = SinuLookupTable::Instance()->SinValue(roll);
  sp = SinuLookupTable::Instance()->SinValue(pitch);
  sy = SinuLookupTable::Instance()->SinValue(yaw);

  R[0][0] = cy * cp;
  R[0][1] = -sy * cr + cy * sr * sp;
  R[0][2] = sy * sr + cy * cr * sp;
  R[1][0] = sy * cp;
  R[1][1] = cy * cr + sy * sr * sp;
  R[1][2] = -cy * sr + sy * cr * sp;
  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

cv::Vec3f Transform::RotationMatrixToEulerAngels(const cv::Mat &R) {
  assert(IsRotationMatrix(R));

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6;  // If

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return cv::Vec3f(x, y, z);
}

void Transform::GaussProjInvCal(double X, double Y, double *longitude,
                                double *latitude) {
  int ProjNo;
  int ZoneWide;  ////带宽

  double longitude1, latitude1, longitude0, X0, Y0, xval, yval;
  double e1, e2, f, a, ee, NN, T, C, M, D, R, u, fai, iPI;

  iPI = 0.0174532925199433;  // 3.1415926535898/180.0;

  a = 6378245.0;
  f = 1.0 / 298.3;  // 54年北京坐标系参数
  // a=6378140.0; f=1/298.257; //80年西安坐标系参数
  //  a=6378137;f=1/298.257223563;  //WGS-84坐标系

  ZoneWide = 6;  // 6度带宽

  ProjNo = (int)(X / 1000000L);  //查找带号

  longitude0 = (ProjNo - 1) * ZoneWide + ZoneWide / 2;
  longitude0 = longitude0 * iPI;  //中央经线
  X0 = ProjNo * 1000000L + 500000L;
  Y0 = 0;

  xval = X - X0;
  yval = Y - Y0;  //带内大地坐标
  e2 = 2 * f - f * f;

  e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
  ee = e2 / (1 - e2);
  M = yval;

  u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));

  fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) +
        (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u)

        + (151 * e1 * e1 * e1 / 96) * sin(6 * u) +
        (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
  C = ee * cos(fai) * cos(fai);
  T = tan(fai) * tan(fai);

  NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));

  R = a * (1 - e2) /
      sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) *
           (1 - e2 * sin(fai) * sin(fai)));
  D = xval / NN;

  //计算经度(Longitude) 纬度(Latitude)

  longitude1 = longitude0 +
               (D - (1 + 2 * T + C) * D * D * D / 6 +
                (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D *
                    D * D * D / 120) /
                   cos(fai);

  latitude1 =
      fai -
      (NN * tan(fai) / R) *
          (D * D / 2 -
           (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24

           +
           (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D *
               D * D * D * D / 720);  //转换为度 DD

  *longitude = longitude1 / iPI;
  *latitude = latitude1 / iPI;
}
