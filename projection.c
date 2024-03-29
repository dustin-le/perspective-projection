// Le, Dustin E.
// dxl0689
// 2019-10-25
//----------------------------------------------------------
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "projection.h"

//----------------------------------------------------------
Projection *allocProjection()
{
  Projection *p;

  p = (Projection *) calloc( 1, sizeof( Projection ) );
  if ( p == NULL ) {
    fprintf( stderr, "allocProjection: Unable to allocate Projection.\n" );
    exit( 1 );
  }

  return p;
}

//----------------------------------------------------------
Projection *computeProjection( View *v )
{
  Projection *p = allocProjection();

  p->m_fx = -(v->m_worldXMin);
  p->m_fy = -(v->m_worldYMin);
  p->m_gx = v->m_width * v->m_viewportXMin;
  p->m_gy = v->m_height * v->m_viewportYMin;
  p->m_sx = (v->m_width * (v->m_viewportXMax - v->m_viewportXMin)) / (v->m_worldXMax - v->m_worldXMin);
  p->m_sy = (v->m_height * (v->m_viewportYMax - v->m_viewportYMin)) / (v->m_worldYMax - v->m_worldYMin);
  p->m_ax = p->m_fx * p->m_sx + p->m_gx;
  p->m_ay = p->m_fy * p->m_sy + p->m_gy;
  p->m_cameraDistance = v->m_cameraDistance;

  return p;
}

//----------------------------------------------------------
void dumpProjection( Projection *p )
{
  printf( "#- Projection parameters ---------------\n" );
  printf( "(fx, fy) : ( %13.6f, %13.6f )\n", p->m_fx, p->m_fy );
  printf( "(gx, gy) : ( %13.6f, %13.6f )\n", p->m_gx, p->m_gy );
  printf( "(sx, sy) : ( %13.6f, %13.6f )\n", p->m_sx, p->m_sy );
  printf( "(ax, ay) : ( %13.6f, %13.6f )\n", p->m_ax, p->m_ay );
  printf( "Camera distance : %13.6f\n", p->m_cameraDistance );
  printf( "#---------------------------------------\n" );
}

//----------------------------------------------------------
void freeProjection( Projection *p )
{
  free( p );
}

//----------------------------------------------------------
void projectVertexList( Projection *p, Vertex *v, int numVertices )
{
  if (p->m_cameraDistance != 0)
  {
    for (int i = 0; i < numVertices; i++)
    {
      double old_x = v[i].x / (1 - v[i].z / p->m_cameraDistance);
      double old_y = v[i].y / (1 - v[i].z / p->m_cameraDistance);

      v[i].x = p->m_sx * old_x + p->m_ax;
      v[i].y = p->m_sy * old_y + p->m_ay;
      v[i].z = 0;
    }
  }

  else
  {
    for (int i = 0; i < numVertices; i++)
    {
      double old_x = v[i].x;
      double old_y = v[i].y;

      v[i].x = p->m_sx * old_x + p->m_ax;
      v[i].y = p->m_sy * old_y + p->m_ay;
      v[i].z = 0;
    }
  }
}

//----------------------------------------------------------
#define DEGREES_TO_RADIANS(d) (M_PI*(d)/180.0)

void rotateVertexList( View *view, Vertex *vertex, int numVertices, Vertex center )
{

  double r00 = cos(DEGREES_TO_RADIANS(view->m_psi)) * cos(DEGREES_TO_RADIANS(view->m_theta));
  double r01 = -(cos(DEGREES_TO_RADIANS(view->m_theta)) * sin(DEGREES_TO_RADIANS(view->m_psi)));
  double r02 = sin(DEGREES_TO_RADIANS(view->m_theta));

  double r10 = cos(DEGREES_TO_RADIANS(view->m_phi)) * sin(DEGREES_TO_RADIANS(view->m_psi)) + cos(DEGREES_TO_RADIANS(view->m_psi)) * sin(DEGREES_TO_RADIANS(view->m_phi)) * sin(DEGREES_TO_RADIANS(view->m_theta));
  double r11 = cos(DEGREES_TO_RADIANS(view->m_phi)) * cos(DEGREES_TO_RADIANS(view->m_psi)) - sin(DEGREES_TO_RADIANS(view->m_phi)) * sin(DEGREES_TO_RADIANS(view->m_psi)) * sin(DEGREES_TO_RADIANS(view->m_theta));
  double r12 = -(cos(DEGREES_TO_RADIANS(view->m_theta)) * sin(DEGREES_TO_RADIANS(view->m_phi)));

  double r20 = -(cos(DEGREES_TO_RADIANS(view->m_phi)) * cos(DEGREES_TO_RADIANS(view->m_psi)) * sin(DEGREES_TO_RADIANS(view->m_theta))) + sin(DEGREES_TO_RADIANS(view->m_phi)) * sin(DEGREES_TO_RADIANS(view->m_psi));
  double r21 = cos(DEGREES_TO_RADIANS(view->m_phi)) * sin(DEGREES_TO_RADIANS(view->m_psi)) * sin(DEGREES_TO_RADIANS(view->m_theta)) + cos(DEGREES_TO_RADIANS(view->m_psi)) * sin(DEGREES_TO_RADIANS(view->m_phi));
  double r22 = cos(DEGREES_TO_RADIANS(view->m_phi)) * cos(DEGREES_TO_RADIANS(view->m_theta));

  double ex = -(r00 * center.x) - (r01 * center.y) - (r02 * center.z) + center.x;
  double ey = -(r10 * center.x) - (r11 * center.y) - (r12 * center.z) + center.y;
  double ez = -(r20 * center.x) - (r21 * center.y) - (r22 * center.z) + center.z;

  for (int i = 0; i < numVertices; i ++)
  {
    double old_x = vertex[i].x;
    double old_y = vertex[i].y;
    double old_z = vertex[i].z;

    vertex[i].x = (r00 * old_x) + (r01 * old_y) + (r02 * old_z) + ex;
    vertex[i].y = (r10 * old_x) + (r11 * old_y) + (r12 * old_z) + ey;
    vertex[i].z = (r20 * old_x) + (r21 * old_y) + (r22 * old_z) + ez;
  }

}

//----------------------------------------------------------

