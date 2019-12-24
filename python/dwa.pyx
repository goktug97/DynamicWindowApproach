# distutils: sources = src/dwa.c
# distutils: include_dirs = src

from libc.stdlib cimport malloc, free

import numpy as np
cimport numpy as np

cimport cdwa

cdef class Velocity:
    cdef cdwa.Velocity* thisptr

    def __cinit__(self, float linear_velocity, float angular_velocity):
        self.thisptr = <cdwa.Velocity*>malloc(sizeof(cdwa.Velocity))
        self.thisptr.linearVelocity = linear_velocity
        self.thisptr.angularVelocity = angular_velocity
        if self.thisptr is NULL:
            raise MemoryError

    def __dealloc__(self):
        if self.thisptr is not NULL:
            free(self.thisptr)

    property linear_velocity:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.linearVelocity

    property angular_velocity:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.angularVelocity

    def __str__(self):
        string = f'Linear Velocity: {self.linear_velocity}'
        string = f'{string}, Angular Velocity: {self.angular_velocity}'
        return string

cdef class Point:
    cdef cdwa.Point* thisptr

    def __cinit__(self, float x, float y):
        self.thisptr = <cdwa.Point*>malloc(sizeof(cdwa.Point))
        self.thisptr.x = x
        self.thisptr.y = y
        if self.thisptr is NULL:
            raise MemoryError

    def __dealloc__(self):
        if self.thisptr is not NULL:
            free(self.thisptr)

    property x:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.x

    property y:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.y

    def __str__(self):
        string = f'x: {self.thisptr.x}, y: {self.thisptr.y}'
        return string

cdef class Pose:
    cdef cdwa.Pose* thisptr

    def __cinit__(self, Point point, float yaw):
        self.thisptr = <cdwa.Pose*>malloc(sizeof(cdwa.Pose))
        self.thisptr.point.x = point.x
        self.thisptr.point.y = point.y
        self.thisptr.yaw = yaw
        if self.thisptr is NULL:
            raise MemoryError

    def __dealloc__(self):
        if self.thisptr is not NULL:
            free(self.thisptr)

    property point:
        def __get__(self):
            assert self.thisptr is not NULL
            return Point(self.thisptr.point.x, self.thisptr.point.y)

    property yaw:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.yaw

    def __str__(self):
        string = f'x: {self.thisptr.point.x}, y: {self.thisptr.point.y}'
        string = f'{string}, yaw: {self.thisptr.yaw}'
        return string

cdef class Config:
    cdef cdwa.Config* thisptr

    def __cinit__(self, float max_speed, float min_speed,
                  float max_yawrate, float max_accel, float max_dyawrate,
                  float velocity_resolution, float yawrate_resolution, float dt,
                  float predict_time, float heading, float clearance, float velocity,
                  list base):

        self.thisptr = <cdwa.Config*>malloc(sizeof(cdwa.Config))
        self.thisptr.maxSpeed = max_speed
        self.thisptr.minSpeed = min_speed
        self.thisptr.maxYawrate = max_yawrate
        self.thisptr.maxAccel = max_accel
        self.thisptr.maxdYawrate = max_dyawrate
        self.thisptr.velocityResolution = velocity_resolution
        self.thisptr.yawrateResolution = yawrate_resolution
        self.thisptr.dt = dt
        self.thisptr.predictTime = predict_time
        self.thisptr.heading = heading
        self.thisptr.clearance = clearance
        self.thisptr.velocity = velocity
        self.thisptr.base.xtop = base[0]
        self.thisptr.base.yleft = base[1]
        self.thisptr.base.xbottom = base[2]
        self.thisptr.base.yright = base[3]
        if self.thisptr is NULL:
            raise MemoryError

    def __dealloc__(self):
        if self.thisptr is not NULL:
            free(self.thisptr)

    property max_speed:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.maxSpeed
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.maxSpeed = value

    property min_speed:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.minSpeed
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.minSpeed = value

    property dt:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.dt
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.dt = value

    property max_yawrate:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.maxYawrate
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.maxYawrate = value

    property max_accel:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.maxAccel
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.maxAccel = value

    property max_dyawrate:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.maxdYawrate
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.maxdYawrate = value

    property velocity_resolution:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.velocityResolution
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.velocityResolution = value

    property yawrate_resolution:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.yawrateResolution
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.yawrateResolution = value

    property predict_time:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.predictTime
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.predictTime = value

    property heading:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.heading
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.heading = value

    property clearance:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.clearance
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.clearance = value

    property velocity:
        def __get__(self):
            assert self.thisptr is not NULL
            return self.thisptr.velocity
        def __set__(self, value):
            assert self.thisptr is not NULL
            self.thisptr.velocity = value

    def __str__(self):
        string = f'maxSpeed               {self.thisptr.maxSpeed}' 
        string = f'{string}\nminSpeed               {self.thisptr.minSpeed}' 
        string = f'{string}\nmaxYawrate             {self.thisptr.maxYawrate}' 
        string = f'{string}\nmaxAccel               {self.thisptr.maxAccel}' 
        string = f'{string}\nmaxdYawrate            {self.thisptr.maxdYawrate}' 
        string = f'{string}\nvelocityResolution     {self.thisptr.velocityResolution}' 
        string = f'{string}\nyawrateResolution      {self.thisptr.yawrateResolution}' 
        string = f'{string}\ndt                     {self.thisptr.dt}' 
        string = f'{string}\npredictTime            {self.thisptr.predictTime}' 
        string = f'{string}\nheading                {self.thisptr.heading}' 
        string = f'{string}\nclearance              {self.thisptr.clearance}' 
        string = f'{string}\nvelocity               {self.thisptr.velocity}' 
        string = f'{string}\nbase.xtop              {self.thisptr.base.xtop}' 
        string = f'{string}\nbase.yleft             {self.thisptr.base.yleft}' 
        string = f'{string}\nbase.xbottom           {self.thisptr.base.xbottom}' 
        string = f'{string}\nbase.yright            {self.thisptr.base.yright}' 
        return string

cdef class PointCloud:
    cdef cdwa.PointCloud* thisptr

    def __cinit__(self, np.ndarray[np.float32_t, ndim=2] point_cloud):
        cdef int size = len(point_cloud)
        self.thisptr = <cdwa.PointCloud*>cdwa.createPointCloud(size)
        self.thisptr.size = size
        cdef float x
        cdef float y
        for i in range(size):
            x, y = point_cloud[i]
            self.thisptr.points[i].x = x
            self.thisptr.points[i].y = y
        if self.thisptr is NULL:
            raise MemoryError

    def __dealloc__(self):
        if self.thisptr is not NULL:
            cdwa.freePointCloud(self.thisptr)

def motion(tuple pose, tuple velocity, float dt):
    cdef float x, y, yaw, v , w
    x, y, yaw = pose
    v, w = velocity
    cdef Pose _pose = Pose(Point(x, y), yaw)
    cdef Velocity _velocity = Velocity(v, w)
    cdef cdwa.Pose new_pose
    new_pose = cdwa.motion(_pose.thisptr[0], _velocity.thisptr[0], dt)
    return (new_pose.point.x, new_pose.point.y, new_pose.yaw)

def planning(tuple pose, tuple velocity, tuple goal,
             np.ndarray[np.float32_t, ndim=2] point_cloud,
             Config config):

    cdef float x, y, yaw, v , w, gx, gy
    cdef PointCloud _point_cloud = PointCloud(point_cloud)
    x, y, yaw = pose
    v, w = velocity
    gx, gy = goal
    cdef Pose _pose = Pose(Point(x, y), yaw)
    cdef Velocity _velocity = Velocity(v, w)
    cdef Point _goal = Point(gx, gy)
    cdef cdwa.Velocity best_velocity
    best_velocity = cdwa.planning(_pose.thisptr[0], _velocity.thisptr[0],
                                  _goal.thisptr[0], _point_cloud.thisptr,
                                  config.thisptr[0]);

    return (best_velocity.linearVelocity, best_velocity.angularVelocity)
