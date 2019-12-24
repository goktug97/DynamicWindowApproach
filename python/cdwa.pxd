cdef extern from "dwa.h":
    ctypedef struct Rect:
        float xtop
        float yleft
        float xbottom
        float yright
    ctypedef struct Config:
        float maxSpeed
        float minSpeed
        float maxYawrate
        float maxAccel
        float maxdYawrate
        float velocityResolution
        float yawrateResolution
        float dt
        float predictTime
        float heading
        float clearance
        float velocity
        Rect base
    ctypedef struct Point:
        float x
        float y
    ctypedef struct PointCloud:
        int size;
        Point *points;
    ctypedef struct Pose:
        Point point
        float yaw
    ctypedef struct Velocity:
        float linearVelocity
        float angularVelocity
    ctypedef struct DynamicWindow:
        int nPossibleV;
        float *possibleV;
        int nPossibleW;
        float *possibleW;

    Pose motion(Pose pose, Velocity velocity, float dt);
    Velocity planning(
        Pose pose, Velocity velocity, Point goal,
        PointCloud *point_cloud, Config config);
    PointCloud* createPointCloud(int size);
    void freePointCloud(PointCloud* pointCloud);
