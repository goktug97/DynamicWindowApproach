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

void main() {
}
