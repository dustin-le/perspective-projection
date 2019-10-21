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

  // TODO: Compute the proper values of fx, fy, gx, gy, sx, sy,
  //       ax, and ay and store them in p->...
  //       Save the camera distance from the view in the
  //       projection.

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
  // TODO: Using the projection parameters in p, traverse the
  //       given list of vertices (there are numVertices of them)
  //       and project each vertex:
  //         1. If camera distance is not 0.0, first do a
  //            perspective adjustment.
  //         2. Once the vertex is adjusted for perspective,
  //            calculate its corresponding screen coordinates.

  if (p->m_cameraDistance != 0)
  {

  }

  for (int i = 0; i < numVertices; i++)
  {
    
  }
}

//----------------------------------------------------------
#define DEGREES_TO_RADIANS(d) (M_PI*(d)/180.0)

void rotateVertexList( View *view, Vertex *vertex, int numVertices, Vertex center )
{
  // TODO: Using the Euler angles given in the view, traverse the
  //       given list of vertices (there are numVertices of them)
  //       and rotate each vertex.
  //
  //       Compute the r00 through r22 values and the ex, ey, ez
  //       values _before_ looping through the vertex list!
  double r00 = cos(view->m_psi) * cos(view->m_theta);
  double r01 = -(cos(view->m_theta) * sin(view->m_psi));
  double r02 = sin(view->m_theta);

  double r10 = cos(view->m_phi) * sin(view->m_psi) + cos(view->m_psi) * sin(view->m_phi) * sin(view->m_theta);
  double r11 = cos(view->m_phi) * cos(view->m_psi) - sin(view->m_phi) * sin(view->m_psi) * sin(view->m_theta);
  double r12 = -(cos(view->m_theta) * sin(view->m_phi));

  double r20 = -(cos(view->m_phi) * cos(view->m_psi) * sin(view->m_theta)) + sin(view->m_phi) * sin(view->m_psi);
  double r21 = cos(view->m_phi) * sin(view->m_psi) * sin(view->m_theta) + cos(view->m_psi) * sin(view->m_phi);
  double r22 = cos(view->m_phi) * cos(view->m_theta);

  double ex = -(r00 * center.x) - (r01 * center.y) - (r02 * center.z) + center.x;
  double ey = -(r10 * center.x) - (r11 * center.y) - (r12 * center.z) + center.y;
  double ez = -(r20 * center.x) - (r21 * center.y) - (r22 * center.z) + center.z;

  for (int i = 0; i < numVertices; i ++)
  {
    vertex[i].x = (r00 * vertex[i].x) + (r01 * vertex[i].y) + (r02 * vertex[i].z) + ex;
    vertex[i].y = (r10 * vertex[i].x) + (r11 * vertex[i].y) + (r12 * vertex[i].z) + ey;
    vertex[i].z = (r20 * vertex[i].x) + (r21 * vertex[i].y) + (r22 * vertex[i].z) + ez;
  }

}

//----------------------------------------------------------

