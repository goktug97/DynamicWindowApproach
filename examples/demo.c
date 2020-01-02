#include "SDL/SDL.h"
#include <GL/gl.h>
#include "dwa.h"

#define RESOLUTION_WIDTH  600
#define RESOLUTION_HEIGHT 600
#define M_PI 3.14159265358979323846
#define M_PI2 M_PI * 2.0

void initGL(void);
void renderRectangle(Rect rect, Pose pose);    
void renderPointCloud(PointCloud *pointCloud, int size);
void drawCircle(GLfloat x, GLfloat y, GLfloat radius);

/*
NOTE: 1 Unit = 0.1 m
That's why everything is multiplied or divided by 10.
*/

void initGL(void) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-RESOLUTION_WIDTH/2, RESOLUTION_WIDTH/2,
          -RESOLUTION_HEIGHT/2, RESOLUTION_HEIGHT/2,
          1, -1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void drawCircle(GLfloat x, GLfloat y, GLfloat radius){
  int nTriangle = 20;
  glBegin(GL_TRIANGLE_FAN);
  glVertex2f(x, y);
  for (int i = 0; i <= nTriangle; i++) { 
    glVertex2f(x + (radius * cos(i * M_PI2 / nTriangle)), 
               y + (radius * sin(i * M_PI2 / nTriangle)));
  }
  glEnd();
}

void renderPointCloud(PointCloud *pointCloud, int size){
  glBegin(GL_POINTS);
  for (size_t i = 0; i < size; i++){
    glColor3f(0, 0, 1);
    drawCircle(pointCloud->points[i].x*10, pointCloud->points[i].y*10, 3.0);
  }
  glEnd();
}

void renderRectangle(Rect rect, Pose pose) {
  glPushMatrix();
  glTranslatef(pose.point.x*10.0,
               pose.point.y*10.0,
               0.0f);
  glRotatef(pose.yaw * 180.0/M_PI, 0, 0, 1);
  glBegin(GL_POLYGON);
  glColor3f(1, 0, 0);
  glVertex2f(rect.xmin*10, rect.ymax*10);
  glVertex2f(rect.xmin*10, rect.ymin*10);
  glVertex2f(rect.xmax*10, rect.ymin*10);
  glVertex2f(rect.xmax*10, rect.ymax*10);
  glEnd();
  glPopMatrix();
}

int main() {
  int running = 1;
  int drawing = 0;

  Point goal;
  goal.x = 0.0;
  goal.y = 0.0;

  PointCloud *tmpPointCloud;
  PointCloud *pointCloud = createPointCloud(100);
  int currentIdx = 0;

  Velocity velocity;
  velocity.linearVelocity = 0.0;
  velocity.angularVelocity = 0.0;

  Pose pose;
  pose.point.x = 0.0;
  pose.point.y = 0.0;
  pose.yaw = 0.0;

  Config config;
  Rect rect;

  rect.xmin = -3.0;
  rect.ymin = -2.5;
  rect.xmax = +3.0;
  rect.ymax = +2.5;

  config.maxSpeed = 5.0;
  config.minSpeed = 0.0;
  config.maxYawrate = 60.0 * M_PI / 180.0;
  config.maxAccel = 15.0;
  config.maxdYawrate = 110.0 * M_PI / 180.0;
  config.velocityResolution = 0.1;
  config.yawrateResolution = 1.0 * M_PI / 180.0;
  config.dt = 0.1;
  config.predictTime = 3.0;
  config.heading = 0.15;
  config.clearance = 1.0;
  config.velocity = 1.0;
  config.base = rect;

  if (SDL_Init(SDL_INIT_EVERYTHING) == -1) return 1;

  SDL_WM_SetCaption("test", NULL);
  SDL_SetVideoMode(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, 0, SDL_OPENGL);

  initGL();

  while (running) {
    SDL_Event sdlEvent;
    glClear(GL_COLOR_BUFFER_BIT);
    while (SDL_PollEvent(&sdlEvent) > 0) {
      switch (sdlEvent.type) {
      case SDL_QUIT:
        running = 0;
        break;
      case SDL_KEYDOWN:
        switch (sdlEvent.key.keysym.sym) {
        case SDLK_ESCAPE:
          running = 0;
        case SDLK_r:
          freePointCloud(pointCloud);
          pointCloud = createPointCloud(100);
          currentIdx = 0;
          pose.point.x = 0.0;
          pose.point.y = 0.0;
          pose.yaw = 0.0;
          velocity.linearVelocity = 0.0;
          velocity.angularVelocity = 0.0;
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        drawing = 1;
        break;
      case SDL_MOUSEBUTTONUP:
        drawing = 0;
        break;
      case SDL_MOUSEMOTION:
        if (drawing) {
          pointCloud->points[currentIdx].x = (sdlEvent.button.x -
                                              RESOLUTION_WIDTH/2.0)/10.0;
          pointCloud->points[currentIdx].y = (-sdlEvent.button.y +
                                              RESOLUTION_HEIGHT/2.0)/10.0;
          currentIdx++;
          if (!(currentIdx % 100)) {
            tmpPointCloud = createPointCloud(currentIdx);
            memcpy(tmpPointCloud->points, pointCloud->points,
                   currentIdx*sizeof(Point));
            freePointCloud(pointCloud);
            pointCloud = createPointCloud(currentIdx+100);
            memcpy(pointCloud->points, tmpPointCloud->points,
                   currentIdx*sizeof(Point));
            freePointCloud(tmpPointCloud);
          }
        } else {
          goal.x = (sdlEvent.button.x - RESOLUTION_WIDTH/2.0)/10.0;
          goal.y = (-sdlEvent.button.y + RESOLUTION_HEIGHT/2.0)/10.0;
          glColor3f(0, 1, 0);
          drawCircle(goal.x*10, goal.y*10, 3);
        }
        break;
      }
    }
    if (currentIdx) {
      tmpPointCloud = createPointCloud(currentIdx);
      memcpy(tmpPointCloud->points, pointCloud->points, currentIdx*sizeof(Point));
      velocity = planning(pose, velocity, goal, tmpPointCloud, config);
      pose = motion(pose, velocity, config.dt);
      freePointCloud(tmpPointCloud);
      renderPointCloud(pointCloud, currentIdx);
    }
    renderRectangle(config.base, pose);
    SDL_GL_SwapBuffers();
  }

  SDL_Quit();
  freePointCloud(pointCloud);
  return 0;
}
