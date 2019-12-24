#include "dwa.h"

void
createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow) {
  float minV = max(config.minSpeed, velocity.linearVelocity - config.maxAccel * config.dt);
  float maxV = min(config.maxSpeed, velocity.linearVelocity + config.maxAccel * config.dt);
  float minW =
    max(-config.maxYawrate, velocity.angularVelocity - config.maxdYawrate * config.dt);
  float maxW =
    max(config.maxYawrate, velocity.angularVelocity + config.maxdYawrate * config.dt);

  int nPossibleV = (maxV - minV) / config.velocityResolution;
  int nPossibleW = (maxW - minW) / config.yawrateResolution;
  *dynamicWindow = malloc(sizeof(DynamicWindow));

  (*dynamicWindow)->possibleV = malloc(nPossibleV * sizeof(float));
  (*dynamicWindow)->possibleW = malloc(nPossibleW * sizeof(float));
  (*dynamicWindow)->nPossibleV = nPossibleV;
  (*dynamicWindow)->nPossibleW = nPossibleW;

  for(int i=0; i < nPossibleV; i++) {
    (*dynamicWindow)->possibleV[i] = minV + (float)i * config.velocityResolution;
  }

  for(int i=0; i < nPossibleW; i++) {
    (*dynamicWindow)->possibleW[i] = minW + (float)i * config.yawrateResolution;
  }
}

void freeDynamicWindow(DynamicWindow *dynamicWindow){
  free(dynamicWindow->possibleV);
  free(dynamicWindow->possibleW);
  free(dynamicWindow);
}

PointCloud* createPointCloud(int size){
  PointCloud* pointCloud = malloc(sizeof(PointCloud));
  pointCloud->points = malloc(size * sizeof(Point));
  pointCloud->size = size;
  return pointCloud;
}

void freePointCloud(PointCloud* pointCloud){
  free(pointCloud->points);
  free(pointCloud);
}

Pose motion(Pose pose, Velocity velocity, float dt){
  Pose new_pose;
  new_pose.yaw = pose.yaw + velocity.angularVelocity * dt;
  new_pose.point.x = pose.point.x + velocity.linearVelocity * cos(new_pose.yaw) * dt;
  new_pose.point.y = pose.point.y + velocity.linearVelocity * sin(new_pose.yaw) * dt;
  return new_pose;
}

float calculateVelocityCost(Velocity velocity, Config config) {
  return config.maxSpeed - velocity.linearVelocity;
}

float calculateHeadingCost(Pose pose, Point goal) {
  float dx = goal.x - pose.point.x;
  float dy = goal.y - pose.point.y;
  float angleError = atan2(dy, dx);
  float angleCost = angleError - pose.yaw;
  return fabs(atan2(sin(angleCost), cos(angleCost)));
}

float
calculateClearanceCost
(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config) {
  Pose pPose = pose;
  float time = 0.0;
  float minr = FLT_MAX;
  float r;
  float dx;
  float dy;

  float x;
  float y;

  while (time < config.predictTime) {
    pPose = motion(pPose, velocity, config.dt);
      
    for(int i = 0; i < pointCloud->size; ++i) {
      dx = pPose.point.x - pointCloud->points[i].x;
      dy = pPose.point.y - pointCloud->points[i].y;
      x = -dx * cos(pPose.yaw) + -dy * sin(pPose.yaw);
      y = -dx * -sin(pPose.yaw) + -dy * cos(pPose.yaw);
      if (x <= config.base.xtop &&
	  x >= config.base.xbottom &&
	  y <= config.base.yleft &&
	  y >= config.base.yright){
	return FLT_MAX;
      }
      r = sqrtf(dx*dx + dy*dy);
      if (r < minr)
	minr = r;
    }
    time += config.dt;
  }
  return 1.0 / minr;
}

Velocity
planning(Pose pose, Velocity velocity, Point goal,
	 PointCloud *pointCloud, Config config){
  DynamicWindow *dw;
  createDynamicWindow(velocity, config, &dw);
  Velocity pVelocity;
  Pose pPose = pose;
  float total_cost = FLT_MAX;
  float cost;
  Velocity bestVelocity;
  for (int i = 0; i < dw->nPossibleV; ++i) {
    for (int j = 0; j < dw->nPossibleW; ++j) {
      pPose = pose;
      pVelocity.linearVelocity = dw->possibleV[i];
      pVelocity.angularVelocity = dw->possibleW[j];
      pPose = motion(pPose, pVelocity, config.predictTime);
      cost = 
	config.velocity * calculateVelocityCost(pVelocity, config) + 
	config.heading * calculateHeadingCost(pPose, goal) +
	config.clearance * calculateClearanceCost(pose, pVelocity, pointCloud, config);
      if (cost < total_cost) {
	total_cost = cost;
	bestVelocity = pVelocity;
      }
    }
  }
  freeDynamicWindow(dw);
  return bestVelocity;
}
