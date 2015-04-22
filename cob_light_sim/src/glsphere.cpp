#include "../include/cob_light_sim/glsphere.hpp"
#include <math.h>
#include <QtOpenGL>

GLSphere::GLSphere(QObject *parent, double r, int divisions) :
  QObject(parent), _radius(r), _divisions(divisions)
{

  // Determine the number of slices and stacks to generate.
  static int const slicesAndStacks[] = {
      8, 4,
      8, 8,
      16, 8,
      16, 16,
      32, 16,
      32, 32,
      64, 32,
      64, 64,
      128, 64,
      128, 128
  };

  if (_divisions < 1)
    _divisions = 1;
  else if (_divisions > 10)
    _divisions = 10;
  int stacks = slicesAndStacks[_divisions * 2 - 1];
  int slices = slicesAndStacks[_divisions * 2 - 2];

  // Precompute sin/cos values for the slices and stacks.
  const int maxSlices = 128 + 1;
  const int maxStacks = 128 + 1;
  qreal sliceSin[maxSlices];
  qreal sliceCos[maxSlices];
  qreal stackSin[maxStacks];
  qreal stackCos[maxStacks];
  for (int slice = 0; slice < slices; ++slice) {
      qreal angle = 2 * M_PI * (slices - 1 - slice) / slices;
      sliceSin[slice] = qFastSin(angle);
      sliceCos[slice] = qFastCos(angle);
  }
  sliceSin[slices] = sliceSin[0]; // Join first and last slice.
  sliceCos[slices] = sliceCos[0];
  for (int stack = 0; stack <= stacks; ++stack) {
      qreal angle = M_PI * stack / stacks;
      stackSin[stack] = qFastSin(angle);
      stackCos[stack] = qFastCos(angle);
  }
  stackSin[0] = 0.0f;             // Come to a point at the poles.
  stackSin[stacks] = 0.0f;

  // Create the stacks.
  for (int stack = 0; stack < stacks; ++stack) {
      qreal z = _radius * stackCos[stack];
      qreal nextz = _radius * stackCos[stack + 1];
      qreal s = stackSin[stack];
      qreal nexts = stackSin[stack + 1];
      qreal c = stackCos[stack];
      qreal nextc = stackCos[stack + 1];
      qreal r = _radius * s;
      qreal nextr = _radius * nexts;
      for (int slice = 0; slice <= slices; ++slice) {
        _vertices.push_back(QVector3D(nextr * sliceSin[slice],
                         nextr * sliceCos[slice], nextz));
          _normals.push_back(QVector3D(sliceSin[slice] * nexts,
                         sliceCos[slice] * nexts, nextc));
          _tex.push_back(QVector2D(1.0f - qreal(slice) / slices,
                         1.0f - qreal(stack + 1) / stacks));

          _vertices.push_back(QVector3D(r * sliceSin[slice],
                         r * sliceCos[slice], z));
          _normals.push_back(QVector3D(sliceSin[slice] * s,
                         sliceCos[slice] * s, c));
          _tex.push_back(QVector2D(1.0f - qreal(slice) / slices,
                         1.0f - qreal(stack) / stacks));
      }
  }
}

GLSphere::~GLSphere()
{

}

void GLSphere::draw() const
{
    glBegin(GL_QUAD_STRIP);
    for(int i = 0; i < _vertices.size(); i+=2)
    {
      glNormal3f(_normals[i].x(),_normals[i].y(),_normals[i].z());
      glVertex3f(_vertices[i].x(),_vertices[i].y(),_vertices[i].z());

      glNormal3f(_normals[i+1].x(),_normals[i+1].y(),_normals[i+1].z());
      glVertex3f(_vertices[i+1].x(),_vertices[i+1].y(),_vertices[i+1].z());
     }
     glEnd();
}
