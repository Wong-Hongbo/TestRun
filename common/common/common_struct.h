#ifndef COMMON_STRUCT_H
#define COMMON_STRUCT_H

#include <vector>
#include "common_types.h"
#include "CommonDefinitionX.hh"

namespace xs {

typedef struct {
  double x;
  double y;
  double z;
} Pose3d;

typedef struct {
  double x;
  double y;
} Pose2d;

typedef struct {
  int x;
  int y;
  int z;
} Pose3i;

typedef struct {
  int x;
  int y;
} Pose2i;

typedef struct {
  Pose3d up;
  Pose3d down;
} Limit;

typedef struct {
  double x;
  double y;
  double z;
  double i;
} PointXYZI;

typedef struct {
  double x;
  double y;
  UINT8 gray;
  int is_obstacle;
} PointGrayValue;

typedef struct  {
  float angle_scope;
  float angle_step;
  float xy_scope;
  float xy_step;
}SearchParams;

typedef struct{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
}Pose6d;

typedef struct{
  double x;
  double y;
  double z;
  double yaw;
  double  azimuth;
  double timestamp;
  bool work_status;
  bool match_state;
  int is_move;
  double score;
  bool init;
  CMD_SOURCE is_auto;
}MapLocalizeData;

typedef struct{
  double timestamp;
  int x;
  int y;
  int yaw;
}LocalPoseData;

}  // namespace xs

#endif  // COMMON_STRUCT_H
