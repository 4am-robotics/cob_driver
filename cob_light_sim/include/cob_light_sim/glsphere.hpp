#ifndef GLSPHERE_H
#define GLSPHERE_H

#include <qobject.h>
#include <qcolor.h>
#include <qvector.h>
#include <qvector2d.h>
#include <qvector3d.h>

class GLSphere : public QObject
 {
 public:
  GLSphere(QObject *parent, double r, int divisions);
  ~GLSphere();

  void draw() const;

 private:
     double _radius;
     int _divisions;
     QVector<QVector3D > _vertices;
     QVector<QVector3D > _normals;
     QVector<QVector2D > _tex;
 };

#endif
