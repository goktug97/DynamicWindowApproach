#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* https://stackoverflow.com/a/3437484 */
 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

/* https://stackoverflow.com/a/3437484 */
 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

typedef struct {
  float xtop;
  float yleft;
  float xbottom;
  float yright;
} Rect;

typedef struct {
  float maxSpeed;
  float minSpeed;
  float maxYawrate;
  float maxAccel;
  float maxdYawrate;
  float velocityResolution;
  float yawrateResolution;
  float dt;
  float predictTime;
  float heading;
  float clearance;
  float velocity;
  Rect base;
} Robot;

typedef struct {
  float linearVelocity;
  float angularVelocity;
} Velocity;

typedef struct {
  float x;
  float y;
} Point;

typedef struct {
  Point point;
  float yaw;
} Pose;

typedef struct {
  int nPossibleV;
  float *possibleV;
  int nPossibleW;
  float *possibleW;
} DynamicWindow;

void
createDynamicWindow(Velocity velocity, Robot robot, DynamicWindow **dynamicWindow) {
  float minV = max(robot.minSpeed, velocity.linearVelocity - robot.maxAccel * robot.dt);
  float maxV = min(robot.maxSpeed, velocity.linearVelocity + robot.maxAccel * robot.dt);
  float minW =
    max(-robot.maxYawrate, velocity.angularVelocity - robot.maxdYawrate * robot.dt);
  float maxW =
    max(robot.maxYawrate, velocity.angularVelocity + robot.maxdYawrate * robot.dt);

  int nPossibleV = (maxV - minV) / robot.velocityResolution;
  int nPossibleW = (maxW - minW) / robot.yawrateResolution;
  *dynamicWindow = malloc(sizeof(DynamicWindow) +
			  nPossibleV * sizeof(float) +
			  nPossibleW * sizeof(float) +
			  2 * sizeof(int));

  (*dynamicWindow)->possibleV = malloc(nPossibleV * sizeof(float));
  (*dynamicWindow)->possibleW = malloc(nPossibleW * sizeof(float));
  (*dynamicWindow)->nPossibleV = nPossibleV;
  (*dynamicWindow)->nPossibleW = nPossibleW;

  for(int i=0; i < (maxV - minV) / robot.velocityResolution; i++) {
    (*dynamicWindow)->possibleV[i] = minV + (float)i * robot.velocityResolution;
  }

  for(int i=0; i < (maxW - minW) / robot.yawrateResolution; i++) {
    (*dynamicWindow)->possibleW[i] = minW + (float)i * robot.yawrateResolution;
  }
}

Pose motion(Pose pose, Velocity velocity, float dt){
  Pose new_pose;
  new_pose.yaw = pose.yaw + velocity.angularVelocity * dt;
  new_pose.point.x = pose.point.x + velocity.linearVelocity * cos(new_pose.yaw) * dt;
  new_pose.point.y = pose.point.y + velocity.linearVelocity * sin(new_pose.yaw) * dt;
  return new_pose;
}

float calculateVelocityCost(Velocity velocity, Robot config) {
  return config.velocity * (config.maxSpeed - velocity.linearVelocity);
}

float calculateHeadingCost(Pose pose, Point goal) {
  float dx = goal.x - pose.point.x;
  float dy = goal.y - pose.point.y;
  float angleError = atan2(dy, dx);
  float angleCost = angleError - pose.yaw;
  return fabs(atan2(sin(angleCost), cos(angleCost)));
}

float
calculateClearanceCost(Pose pose, Velocity velocity, Point *pointCloud, Robot config) {
  Pose pPose = pose;
  float clearanceCost;
  float time = 0.0;
  while (time < config.predictTime) {
    pPose = motion(pPose, velocity, config.dt);
    time += config.dt;
  }
  return clearanceCost;
}

Velocity
planning(Pose pose, Velocity velocity, Point goal, Point *pointCloud, Robot config){
  DynamicWindow *dw;
  createDynamicWindow(velocity, config, &dw);
  free(dw);
  Velocity pVelocity;
  Pose pPose = pose;
  for (int i = 0; i < dw->nPossibleV; ++i) {
    for (int j = 0; j < dw->nPossibleW; ++j) {
      pPose = motion(pPose, pVelocity, config.predictTime);
      pVelocity.linearVelocity = dw->possibleV[i];
      pVelocity.angularVelocity = dw->possibleW[j];
      calculateVelocityCost(pVelocity, config);
      calculateHeadingCost(pPose, goal);
      calculateClearanceCost(pose, pVelocity, pointCloud, config);
    }
  }
  return velocity;
}

void main() {
  Robot config;
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
  goal.y = 0.0;

  Point pointCloud[10];
  pointCloud[0].x = 0.0; pointCloud[0].y = 1.0;
  pointCloud[1].x = 2.0; pointCloud[1].y = 2.0;
  pointCloud[2].x = 0.0; pointCloud[2].y = 3.0;
  pointCloud[3].x = 3.0; pointCloud[3].y = 1.0;
  pointCloud[4].x = 3.5; pointCloud[4].y = 4.0;
  pointCloud[5].x = 5.0; pointCloud[5].y = 4.0;
  pointCloud[6].x = 7.0; pointCloud[6].y = 1.0;
  pointCloud[7].x = 9.0; pointCloud[7].y = 1.0;
  pointCloud[8].x = 8.0; pointCloud[8].y = 8.0;
  pointCloud[9].x = 10.0; pointCloud[9].y = 3.0;

  pose = motion(pose, velocity, config.dt);
  printf("%f %f %f\n", pose.point.x, pose.point.y, pose.yaw);

  planning(pose, velocity, goal, pointCloud, config);
}
