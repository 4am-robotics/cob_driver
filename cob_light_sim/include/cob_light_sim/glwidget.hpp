#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <cob_light/ColorRGBAArray.h>
#include "glsphere.hpp"

class GLWidget : public QGLWidget
{
  Q_OBJECT
public:
  explicit GLWidget(QWidget* parent = 0);
  ~GLWidget();

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

protected:
  void initializeGL();
  void paintGL();
  void resizeGL(int width, int height);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);

public Q_SLOTS:
  void setColors(cob_light::ColorRGBAArray colors);
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);

Q_SIGNALS:
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);

private:
  int xRot;
  int yRot;
  int zRot;
  QPoint lastPos;

  GLSphere* _sphere;


};

#endif // GLWIDGET_H
