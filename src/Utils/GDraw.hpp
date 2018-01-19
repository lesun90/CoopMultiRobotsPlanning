#ifndef MP_GDRAW_HPP_
#define MP_GDRAW_HPP_

#include "Utils/GMaterial.hpp"

#define GL_GLEXT_PROTOTYPES
#include <GL/glu.h>
#include <GL/glext.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "Utils/glutForWindows.h"
#else
#include <GL/glut.h>
#endif


namespace MP
{
    enum
	{
	    GDRAW_TOP  = 1,
	    GDRAW_BASE = 2,
	    GDRAW_SIDE = 4,
	    GDRAW_ALL  = GDRAW_TOP | GDRAW_BASE | GDRAW_SIDE
	};

    enum GDrawParam
	{
	    GDRAW_MINX = 0,
	    GDRAW_MINY,
	    GDRAW_MAXX,
	    GDRAW_MAXY,
	    GDRAW2D,
	    GDRAW_ARROW2D_LENGTH,
	    GDRAW_ARROW2D_HEIGHT,
	    GDRAW_PERSPECTIVE3D_ANGLE,
	    GDRAW_PERSPECTIVE3D_NEAR_PLANE,
	    GDRAW_PERSPECTIVE3D_FAR_PLANE,
	    GDRAW_ORTHO_NEAR_PLANE,
	    GDRAW_ORTHO_FAR_PLANE,
	    GDRAW_ARROW3D_TIP_LENGTH,
	    GDRAW_ARROW3D_TIP_RADIUS,
	    GDRAW_ARROW3D_BODY_RADIUS,
	    GDRAW_USE3D,
	    GDRAW_NR_STACKS,
	    GDRAW_NR_SLICES,
	    GDRAW_COMPONENTS,
	    GDRAW_NR_PARAMS
	};

    double GDrawGetParam(const GDrawParam p);

    void GDrawSetParam(const GDrawParam p, const double val);

    static inline bool GDrawIs2D(void)
    {
	return GDrawGetParam(GDRAW2D) != 0;
    }

    void GDraw2D(void);
    void GDraw3D(void);


    void GDrawWireframe(const bool val);

    void GDrawColor(const double r, const double g, const double b);
    void GDrawColorTrans(const double r, const double g, const double b, const double a);

    static inline void GDrawColor(const double * const rgb)
    {
	GDrawColor(rgb[0], rgb[1], rgb[2]);
    }

    void GDrawIndexColor(const int i);

    void GDrawColorMaterial(const double r, const double g, const double b);

    void GDrawLineWidth(const double w);


    void GDrawIllumination(void);

     void GDrawMaterial(const GMaterial * const mat);

    void GDrawPushTransformation(void);
    void GDrawMultTrans(const double x, const double y, const double z);
    void GDrawMultTrans(const double T[]);
    void GDrawMultRot(const double R[]);
    void GDrawMultFromZAxisToAxis(const double nx, const double ny, const double nz);
    void GDrawMultFromZAxisToAxis(const double norm[]);
    void GDrawMultTransRot(const double x, const double y, const double z, const double R[]);
    void GDrawMultTransRot(const double T[], const double R[]);
    void GDrawScale(const double sx, const double sy, const double sz);
    void GDrawScale(const double s[]);
    void GDrawScale(const double ox, const double oy, const double oz,
		    const double sx, const double sy, const double sz);
    void GDrawScale(const double o[], const double s[]);
    void GDrawPopTransformation(void);

//2D stuff
    void GDrawString2D(const char s[], const double x, const double y);

    void GDrawPoint2D(const double x, const double y);

    void GDrawPoint2D(const double p[]);

    void GDrawPoints2D(const int n, const double pts[]);

    void GDrawSegment2D(const double x1, const double y1,
			const double x2, const double y2);

    void GDrawSegment2D(const double p1[], const double p2[]);

    void GDrawSegment2D(const double p1p2[]);

    void GDrawArrow2D(const double x1, const double y1,
		      const double x2, const double y2);

    void GDrawArrow2D(const double p1[], const double p2[]);

    void GDrawArrow2D(const double p1p2[]);


    void GDrawFrame2D(const double ox, const double oy,
		      const double ax, const double ay,
		      const double bx, const double by);

    void GDrawFrame2D(const double origin[],
		      const double a[],
		      const double b[]);

    void GDrawFrame2D(const double ox, const double oy,
		      const double ax, const double ay);

    void GDrawFrame2D(const double origin[],
		      const double a[]);

    void GDrawTriangle2D(const double x1, const double y1,
			 const double x2, const double y2,
			 const double x3, const double y3);

    void GDrawTriangle2D(const double p1[],
			 const double p2[],
			 const double p3[]);

    void GDrawTriangle2D(const double p1p2p3[]);

    void GDrawAABox2D(const double minx, const double miny,
		      const double maxx, const double maxy);

    void GDrawAABox2D(const double min[], const double max[]);

    void GDrawAABox2D(const double minmax[]);

    void GDrawQuad2D(const double ax, const double ay,
		     const double bx, const double by,
		     const double cx, const double cy,
		     const double dx, const double dy);

    void GDrawQuad2D(const double a[],
		     const double b[],
		     const double c[],
		     const double d[]);

    void GDrawQuad2D(const double abcd[]);


    void GDrawCircle2D(const double cx, const double cy, const double r);

    static inline
    void GDrawCircle2D(const double center[2], const double r)
    {
	GDrawCircle2D(center[0], center[1], r);
    }

    void GDrawRegularPolygon2D(const double cx, const double cy, const double r, const int nsides);

    static inline
    void GDrawRegularPolygon2D(const double center[2], const double r, const int nsides)
    {
	GDrawRegularPolygon2D(center[0], center[1], r, nsides);
    }


    void GDrawConvexPolygon2D(const int n, const double vertices[], const double heights[] = NULL);

    void GDrawPolygon2D(const int nv, const double vertices[], const int ntri, const int triangles[], const double heights[] = NULL);

    void GDrawTriangles2D(const int n, const double vtris[]);

    void GDrawGrid2D(const double xmin,
		     const double ymin,
		     const double xmax,
		     const double ymax,
		     const int    xNrDims,
		     const int    yNrDims);

//3D stuff
    void GDrawString3D(const char s[], const double x, const double y, const double z,
		       const bool rotate, const double scale);

    void GDrawPoint3D(const double x, const double y, const double z);

    void GDrawPoint3D(const double p[]);

    void GDrawSegment3D(const double x1, const double y1, const double z1,
			const double x2, const double y2, const double z2);

    void GDrawSegment3D(const double p1[], const double p2[]);

    void GDrawSegment3D(const double p1p2[]);


    void GDrawArrow3D(const double x1, const double y1, const double z1,
		      const double x2, const double y2, const double z2);

    void GDrawArrow3D(const double p1[], const double p2[]);

    void GDrawArrow3D(const double p1p2[]);

    void GDrawFrame3D(const double ox, const double oy, const double oz,
		      const double ax, const double ay, const double az,
		      const double bx, const double by, const double bz,
		      const double cx, const double cy, const double cz);

    void GDrawFrame3D(const double origin[],
		      const double xaxis[],
		      const double yaxis[],
		      const double zaxis[]);

    void GDrawFrame3D(const double ox, const double oy, const double oz,
		      const double ax, const double ay, const double az,
		      const double bx, const double by, const double bz);

    void GDrawFrame3D(const double origin[],
		      const double xaxis[],
		      const double yaxis[]);

    void GDrawFrame3D(const double origin[],
		      const double R[],
		      const double scale = 0.1);


    void GDrawTriangle3D(const double ax, const double ay, const double az,
			 const double bx, const double by, const double bz,
			 const double cx, const double cy, const double cz);

    void GDrawTriangle3D(const double v1[3],
			 const double v2[3],
			 const double v3[3]);

    void GDrawTriangles3D(const int n, const double tv[]);

    void GDrawParallelogram3D(const double ox, const double oy, const double oz,
			      const double ax, const double ay, const double az,
			      const double bx, const double by, const double bz);


    void GDrawParallelogram3D(const double o[3],
			      const double a[3],
			      const double b[3]);

    void GDrawSphere3D(const double cx, const double cy, const double cz, const double r);

    void GDrawSphere3D(const double c[], const double r);

    void GDrawSphere3D(const double r);

    void GDrawEllipsoid3D(const double cx, const double cy, const double cz,
			  const double rx, const double ry, const double rz);

    void GDrawEllipsoid3D(const double c[], const double rx, const double ry, const double rz);


    void GDrawEllipsoid3D(const double rx,  const double ry,   const double rz);

    void GDrawTorus3D(const double cx, const double cy, const double cz,
		      const double rin, const double rout);

    void GDrawTorus3D(const double c[], const double rin, const double rout);

    void GDrawTorus3D(const double rin, const double rout);

//rectangular cuboid, i.e., box
    void GDrawBox3D(const double dimx, const double dimy, const double dimz);

    void GDrawBox3D(const double dims[]);

    void GDrawBox3D(const double minx, const double miny, const double minz,
		    const double maxx, const double maxy, const double maxz);

    void GDrawBox3D(const double min[], const double max[]);

//parallelepiped
    void GDrawParallelepiped3D(const double cx, const double cy, const double cz,
			       const double x0, const double x1, const double x2,
			       const double y0, const double y1, const double y2,
			       const double z0, const double z1, const double z2);

    void GDrawParallelepiped3D(const double c[],
			       const double xaxis[],
			       const double yaxis[],
			       const double zaxis[]);

    void GDrawParallelepiped3D(const double c[],
			       const double uxaxis[],
			       const double uyaxis[],
			       const double uzaxis[],
			       const double dims[]);

//pyramid
    void GDrawPyramid3D(const int    n,
			const double base_xy[],
			const double px,
			const double py,
			const double pz,
			const double ox_scale,
			const double oy_scale,
			const double basex_scale,
			const double basey_scale);

    void GDrawPyramid3D(const int    n,
			const double base_xy[],
			const double px,
			const double py,
			const double pz,
			const double basex_scale,
			const double basey_scale);

    void GDrawPyramid3D(const int    n,
			const double base_xy[],
			const double px,
			const double py,
			const double pz);

    void GDrawCappedPyramid3D(const int    n,
			      const double base_xy[],
			      const double ax,
			      const double ay,
			      const double az,
			      const double ox_scale,
			      const double oy_scale,
			      const double basex_scale,
			      const double basey_scale,
			      const double topx_scale,
			      const double topy_scale);

    void GDrawCappedPyramid3D(const int    n,
			      const double base_xy[],
			      const double ax,
			      const double ay,
			      const double az,
			      const double basex_scale,
			      const double basey_scale,
			      const double topx_scale,
			      const double topy_scale);

    void GDrawCappedPyramid3D(const int    n,
			      const double base_xy[],
			      const double ax,
			      const double ay,
			      const double az);
//prism
    void GDrawPrism3D(const int    n,
		      const double base_xy[],
		      const double ax,
		      const double ay,
		      const double az,
		      const double ox_scale,
		      const double oy_scale,
		      const double basex_scale,
		      const double basey_scale);


    void GDrawPrism3D(const int    n,
		      const double base_xy[],
		      const double ax,
		      const double ay,
		      const double az,
		      const double basex_scale,
		      const double basey_scale);

    void GDrawPrism3D(const int    n,
		      const double base_xy[],
		      const double ax,
		      const double ay,
		      const double az);
//cylinder
    void GDrawCylinder3D(const double r,
			 const double h);

    void GDrawCylinder3D(const double rx,
			 const double ry,
			 const double ax,
			 const double ay,
			 const double az);

    void GDrawCylinder3D(const double r,
			 const double x1, const double y1, const double z1,
			 const double x2, const double y2, const double z2);

//cone
    void GDrawCone3D(const double r,
		     const double h);

    void GDrawCone3D(const double r,
		     const double px,
		     const double py,
		     const double pz);

    void GDrawCone3D(const double rx,
		     const double ry,
		     const double px,
		     const double py,
		     const double pz);

    void GDrawCappedCone3D(const double rbase,
			   const double rtop,
			   const double h);


    void GDrawCappedCone3D(const double base_rx,
			   const double base_ry,
			   const double top_rx,
			   const double top_ry,
			   const double ax,
			   const double ay,
			   const double az);

    char* GDrawShaderLoadSource(const char *filePath);
    GLuint GDrawShaderCompileFromFile(GLenum type, const char *filePath);
    void GDrawShaderAttachFromFile(GLuint program, GLenum type, const char *filePath);
    void GDrawShaderSceneInit(void);
    void GDrawShaderSceneRender(void);



}

#endif
