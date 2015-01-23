#include "../include/cob_light_sim/glwidget.hpp"

#include <QtOpenGL>
#include <math.h>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

GLWidget::GLWidget(QWidget* parent) :
  QGLWidget(parent), xRot(0), yRot(0), zRot(0)
{
  _sphere = new GLSphere(this, 0.01, 5);
}

GLWidget::~GLWidget()
{
  delete _sphere;
}

QSize GLWidget::minimumSizeHint() const
{
   return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
   return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
   while (angle < 0)
       angle += 360 * 16;
   while (angle > 360 * 16)
       angle -= 360 * 16;
}

void GLWidget::setXRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != xRot) {
       xRot = angle;
       Q_EMIT xRotationChanged(angle);
       updateGL();
   }
}

void GLWidget::setYRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != yRot) {
       yRot = angle;
       Q_EMIT yRotationChanged(angle);
       updateGL();
   }
}

void GLWidget::setZRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != zRot) {
       zRot = angle;
       Q_EMIT zRotationChanged(angle);
       updateGL();
   }
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
   lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
   int dx = event->x() - lastPos.x();
   int dy = event->y() - lastPos.y();

   if (event->buttons() & Qt::LeftButton) {
       setXRotation(xRot + 8 * dy);
       setYRotation(yRot + 8 * dx);
   } else if (event->buttons() & Qt::RightButton) {
       setXRotation(xRot + 8 * dy);
       setZRotation(zRot + 8 * dx);
   }
   lastPos = event->pos();
}

void GLWidget::initializeGL()
{
  glClearColor(0.2,0.2,0.2,1);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_COLOR_MATERIAL);
  static GLfloat lightPosition[4] = { 0.0, 0.0, 7.0, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void GLWidget::paintGL()
 {
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     glLoadIdentity();
     glTranslatef(0.0, 0.0, -15.0);
     glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
     glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
     glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

     glColor3d(1,0,0);
     glTranslatef(0.1, 0.0, 0.0);
     _sphere->draw();
     glTranslatef(0.1, 0.0, 0.0);
     _sphere->draw();
     glTranslatef(0.1, 0.0, 0.0);
     _sphere->draw();
 }

void GLWidget::resizeGL(int width, int height)
{
   int side = qMin(width, height);
   glViewport((width - side) / 2, (height - side) / 2, side, side);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
#ifdef QT_OPENGL_ES_1
   glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#else
   glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#endif
   glMatrixMode(GL_MODELVIEW);
}

void GLWidget::setColors(cob_light::ColorRGBAArray colors)
{

}


