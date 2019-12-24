#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* https://stackoverflow.com/a/3437484 */
#define max(a,b)				\
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a > _b ? _a : _b; })

#define min(a,b)				\
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
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
} Config;

typedef struct {
  float linearVelocity;
  float angularVelocity;
} Velocity;

typedef struct {
  float x;
  float y;
} Point;

typedef struct {
  int size;
  Point *points;
} PointCloud;

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
createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow);
Pose motion(Pose pose, Velocity velocity, float dt);
float calculateVelocityCost(Velocity velocity, Config config);
float calculateHeadingCost(Pose pose, Point goal);
float
calculateClearanceCost
(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config);
Velocity
planning
(Pose pose, Velocity velocity, Point goal, PointCloud *pointCloud, Config config);
PointCloud* createPointCloud(int size);
void freePointCloud(PointCloud* pointCloud);
void freeDynamicWindow(DynamicWindow *dynamicWindow);
