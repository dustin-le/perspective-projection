// Le, Dustin E.
// dxl0689
// 2019-10-25

#if !defined( __VIEW_H__ )
#define __VIEW_H__

//----------------------------------------------------------
typedef struct _View {
  int m_width;
  int m_height;

  double m_cameraDistance;

  double m_worldXMin;
  double m_worldXMax;
  double m_worldYMin;
  double m_worldYMax;

  double m_viewportXMin;
  double m_viewportXMax;
  double m_viewportYMin;
  double m_viewportYMax;

  double m_phi; // Φ
  double m_theta; // Θ
  double m_psi; // Ψ
} View;

//----------------------------------------------------------
View *allocView();
void  dumpView( View *view );
void  freeView( View *view );
View *loadView( char *fName );

//----------------------------------------------------------

#endif

