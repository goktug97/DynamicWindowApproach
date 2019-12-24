#include "dwa.h"

int main() {
  Config config;
  Rect rect;

  rect.xtop = 0.6;
  rect.yleft = 0.5;
  rect.xbottom = -0.6;
  rect.yright = -0.5;

  config.maxSpeed = 0.5;
  config.minSpeed = 0.0;
  config.maxYawrate = 40.0 * M_PI / 180.0;
  config.maxAccel = 15.0;
  config.maxdYawrate = 110.0 * M_PI / 180.0;
  config.velocityResolution = 0.1;
  config.yawrateResolution = 1.0 * M_PI / 180.0;
  config.dt = 0.1;
  config.predictTime = 1.0;
  config.heading = 0.5;
  config.clearance = 0.5;
  config.velocity = 0.5;
  config.base = rect;

  Velocity velocity;
  velocity.linearVelocity = 0.0;
  velocity.angularVelocity = 0.0;

  Pose pose;
  pose.point.x = 0.0;
  pose.point.y = 0.0;
  pose.yaw = 0.0;

  Point goal;
  goal.x = 10.0;
  goal.y = 10.0;

  PointCloud *pointCloud = createPointCloud(3);
  pointCloud->points[0].x = -1.0; pointCloud->points[0].y = 1.0;
  pointCloud->points[1].x = 0.0; pointCloud->points[1].y = 2.0;
  pointCloud->points[2].x = 4.0; pointCloud->points[2].y = 2.0;

  while(1){
    velocity = planning(pose, velocity, goal, pointCloud, config);
    pose = motion(pose, velocity, config.dt);
    printf("x: %f y: %f yaw: %f\n", pose.point.x, pose.point.y, pose.yaw);
    if (sqrtf(powf(pose.point.x-goal.x, 2) + powf(pose.point.y-goal.y, 2)) < 1.0)
      break;
  }
  freePointCloud(pointCloud);
  return 0;
}
