#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/PrintMsg.hpp"
#include <cstdlib>
#include <cmath>

namespace MP
{
    GManager *m_gManager = NULL;

    void GManager::CallbackEventOnActiveMouseMove(int x, int y)
    {
	if(m_gManager)
	{
	    if(m_gManager->HandleEventOnActiveMouseMove(x, y))
		glutPostRedisplay();
	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }

    void GManager::CallbackEventOnPassiveMouseMove(int x, int y)
    {
	if(m_gManager)
	{
	    if(m_gManager->HandleEventOnPassiveMouseMove(x, y))
		glutPostRedisplay();
	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }

    void GManager::CallbackEventOnMouse(int button, int state, int x, int y)
    {
	if(m_gManager)
	{
	    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN &&
	       m_gManager->HandleEventOnMouseLeftBtnDown(x, y))
		glutPostRedisplay();

	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }

    void GManager::CallbackEventOnTimer(int id)
    {
	if(m_gManager)
	{
	    m_gManager->HandleEventOnTimer();
	    glutTimerFunc(m_gManager->m_timer, CallbackEventOnTimer, id);
	    glutPostRedisplay();
	}
    }

    void GManager::CallbackEventOnIdle(void)
    {
	if(m_gManager)
	{
	    if(m_gManager->HandleEventOnIdle())
		glutPostRedisplay();
	}
    }

    void GManager::CallbackEventOnMenu(int item)
    {
	if(m_gManager && m_gManager->HandleEventOnMenu(item))
	    glutPostRedisplay();
    }

    void GManager::CallbackEventOnSpecialKeyPress(int key, int x, int y)
    {
	if(m_gManager &&
	   m_gManager->HandleEventOnSpecialKeyPress(key))
	    glutPostRedisplay();
    }

    void GManager::CallbackEventOnNormalKeyPress(unsigned char key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnNormalKeyPress(key))
	    glutPostRedisplay();
    }


    void GManager::CallbackEventOnSpecialKeyUp(int key, int x, int y)
    {
	if(m_gManager &&
	   m_gManager->HandleEventOnSpecialKeyUp(key))
	    glutPostRedisplay();
    }

    void GManager::CallbackEventOnNormalKeyUp(unsigned char key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnNormalKeyUp(key))
	    glutPostRedisplay();
    }


    void  GManager::CallbackEventOnDisplay(void)
    {
	if(m_gManager)
	{
	    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	    glClearDepth(1.0);
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	    glShadeModel(GL_SMOOTH);

	    glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	    glMatrixMode(GL_PROJECTION);
	    glLoadIdentity();
	    if(m_gManager->m_gCameraAction & ACTION_CAMERA_PROJECTION)
		gluPerspective(GDrawGetParam(GDRAW_PERSPECTIVE3D_ANGLE),
			       (double) glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT),
			       GDrawGetParam(GDRAW_PERSPECTIVE3D_NEAR_PLANE),
			       GDrawGetParam(GDRAW_PERSPECTIVE3D_FAR_PLANE));
	    else
		glOrtho(m_gManager->m_viewZoom * GDrawGetParam(GDRAW_MINX),
			m_gManager->m_viewZoom * GDrawGetParam(GDRAW_MAXX),
			m_gManager->m_viewZoom * GDrawGetParam(GDRAW_MINY),
			m_gManager->m_viewZoom * GDrawGetParam(GDRAW_MAXY),
			GDrawGetParam(GDRAW_ORTHO_NEAR_PLANE),
			GDrawGetParam(GDRAW_ORTHO_FAR_PLANE));

	    glMatrixMode(GL_MODELVIEW);
	    glLoadIdentity();

	    double m[16];
	    m_gManager->m_gCamera.GetModelViewMatrixOpenGL(m);
	    glMultMatrixd(m);


	    if(m_gManager->m_gDrawAction & ACTION_DRAW_TEXTURES)
		glEnable(GL_TEXTURE_2D);
	    else
		glDisable(GL_TEXTURE_2D);

	    if(m_gManager->m_gDrawAction & ACTION_DRAW_LIGHTING)
	    {
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);
		glEnable(GL_LIGHTING);
		GDrawIllumination();
	    }
	    else
		glDisable(GL_LIGHTING);

	    GDrawWireframe((m_gManager->m_gDrawAction & ACTION_DRAW_WIREFRAME) != 0);

//	    GDrawShaderSceneRender();

	    m_gManager->HandleEventOnDisplay();

	    if(m_gManager->m_gDrawAction & ACTION_DRAW_EXPORT_FRAMES)
		ExportFrameAsImage();

//	    glUseProgram(0);


	    glutSwapBuffers();
	}
    }

    GManager::GManager(void)
    {
	m_idWindow      = -1;
	m_timer         =  1;
	m_gCameraAction = ACTION_CAMERA_ROTATE_CENTER |
	    ACTION_CAMERA_ROTATE_GLOBAL_AXIS |
	    ACTION_CAMERA_ROTATE_X |
//	    ACTION_CAMERA_ROTATE_Y |
	    ACTION_CAMERA_ROTATE_Z |
	    ACTION_CAMERA_MOVE_X |
	    ACTION_CAMERA_MOVE_Y |
	    ACTION_CAMERA_MOVE_Z |
	    ACTION_CAMERA_PROJECTION;


	m_gCameraMove   = 0.50;
	m_gDrawAction   = ACTION_DRAW_TEXTURES | ACTION_DRAW_LIGHTING;


	m_mousePrevX    = 0;
	m_mousePrevY    = 0;
	m_viewZoom = 1;

	m_frames = 0;

	m_nrMenuItems                  = 0;
	MENU_CAMERA_ROTATE_CENTER      = ++m_nrMenuItems;
	MENU_CAMERA_ROTATE_GLOBAL_AXIS = ++m_nrMenuItems;
	MENU_CAMERA_ROTATE_X           = ++m_nrMenuItems;
	MENU_CAMERA_ROTATE_Y           = ++m_nrMenuItems;
	MENU_CAMERA_ROTATE_Z           = ++m_nrMenuItems;
	MENU_CAMERA_MOVE_X             = ++m_nrMenuItems;
	MENU_CAMERA_MOVE_Y             = ++m_nrMenuItems;
	MENU_CAMERA_MOVE_Z             = ++m_nrMenuItems;
	MENU_CAMERA_PROJECTION         = ++m_nrMenuItems;
	MENU_CAMERA_STANDARD           = ++m_nrMenuItems;
	MENU_DRAW_TEXTURE              = ++m_nrMenuItems;
	MENU_DRAW_LIGHTING             = ++m_nrMenuItems;
	MENU_DRAW_WIREFRAME            = ++m_nrMenuItems;
	MENU_DRAW_EXPORT_FRAMES        = ++m_nrMenuItems;
	MENU_DRAW_XY_BBOX              = ++m_nrMenuItems;


//	m_gCamera.RotateAroundAxisAtPoint(0.5 * M_PI, 0, 1, 0, 0, 0, 0);

    }

    GManager::~GManager(void)
    {
	if(m_idWindow >= 0)
	    glutDestroyWindow(m_idWindow);
    }


    void GManager::MainLoop(const char * const title, const int width, const int height)
    {
	m_gManager = this;

	static int    argc = 1;
	static char  *args = (char*)"args";

	glutInit(&argc, &args);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	m_idWindow = glutCreateWindow(title);

	glutDisplayFunc(CallbackEventOnDisplay);
	glutMouseFunc(CallbackEventOnMouse);
	glutMotionFunc(CallbackEventOnActiveMouseMove);
	glutPassiveMotionFunc(CallbackEventOnPassiveMouseMove);
	glutIdleFunc(CallbackEventOnIdle);
	glutTimerFunc(0, CallbackEventOnTimer, 0);
	glutKeyboardFunc(CallbackEventOnNormalKeyPress);
	glutSpecialFunc(CallbackEventOnSpecialKeyPress);
	glutKeyboardUpFunc(CallbackEventOnNormalKeyUp);
	glutSpecialUpFunc(CallbackEventOnSpecialKeyUp);


	MenuOptions();

	//RandomSeed();

//	GDrawShaderSceneInit();


	glutMainLoop();
    }

    void GManager::MenuOptions(void)
    {
	m_idMenuCamera = CreateMenu();
	AddMenuEntry("Rotate at center[on]/eye[off]", MENU_CAMERA_ROTATE_CENTER);
	AddMenuEntry("Rotate axis global[on]/camera[off]", MENU_CAMERA_ROTATE_GLOBAL_AXIS);
	AddMenuEntry("Rotation around x-axis [on]", MENU_CAMERA_ROTATE_X);
	AddMenuEntry("Rotation around y-axis [off]", MENU_CAMERA_ROTATE_Y);
	AddMenuEntry("Rotation around z-axis [on]", MENU_CAMERA_ROTATE_Z);
	AddMenuEntry("Moving along x-axis [on]", MENU_CAMERA_MOVE_X);
	AddMenuEntry("Moving along y-axis [on]", MENU_CAMERA_MOVE_Y);
	AddMenuEntry("Moving along z-axis [on]", MENU_CAMERA_MOVE_Z);
	AddMenuEntry("Perspective[on]/orthogonal[off]", MENU_CAMERA_PROJECTION);
	AddMenuEntry("Look at standard position", MENU_CAMERA_STANDARD);

	m_idMenuDraw = CreateMenu();
	AddMenuEntry("Textures [on]", MENU_DRAW_TEXTURE);
	AddMenuEntry("Lighting [on]", MENU_DRAW_LIGHTING);
	AddMenuEntry("Wireframe [off]", MENU_DRAW_WIREFRAME);
	AddMenuEntry("Export frames [off]", MENU_DRAW_EXPORT_FRAMES);
	AddMenuEntry("Set xy bounding box", MENU_DRAW_XY_BBOX);


	m_idMenuMain = CreateMenu();
	AddSubMenu("Camera", m_idMenuCamera);
	AddSubMenu("Draw", m_idMenuDraw);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
    }

    bool GManager::HandleEventOnMenu(const int item)
    {
	if(item == MENU_CAMERA_STANDARD)
	{
	    m_gCamera.SetLookAtStandardPosition();
	    return true;
	}
	else if(item >= MENU_CAMERA_ROTATE_CENTER && item <= MENU_CAMERA_PROJECTION)
	{
	    const int  offset = item - MENU_CAMERA_ROTATE_CENTER;
	    const int  val    = 1 << offset;
	    char msg[50];

	    SetMenu(m_idMenuCamera);
	    if(m_gCameraAction & val)
	    {
		m_gCameraAction &= ~val;
		if(item == MENU_CAMERA_ROTATE_CENTER)
		    sprintf(msg, "Rotate at center[off]/eye[on]");
		else if(item == MENU_CAMERA_ROTATE_GLOBAL_AXIS)
		    sprintf(msg, "Rotate axis global[off]/camera[on]");
		else if(item >= MENU_CAMERA_ROTATE_X && item <= MENU_CAMERA_ROTATE_Z)
		    sprintf(msg, "Rotation around %c-axis [off]", "xyz"[offset - 2]);
		else if(item >= MENU_CAMERA_MOVE_X && item <= MENU_CAMERA_MOVE_Z)
		    sprintf(msg, "Moving along %c-axis [off]", "xyz"[offset - 5]);
		else if(item == MENU_CAMERA_PROJECTION)
		    sprintf(msg, "Perspective[off]/orthogonal[on]");
	    }
	    else
	    {
		m_gCameraAction |= val;
		if(item == MENU_CAMERA_ROTATE_CENTER)
		    sprintf(msg, "Rotate at center[on]/eye[off]");
		else if(item == MENU_CAMERA_ROTATE_GLOBAL_AXIS)
		    sprintf(msg, "Rotate axis global[on]/camera[off]");
		else if(item >= MENU_CAMERA_ROTATE_X && item <= MENU_CAMERA_ROTATE_Z)
		    sprintf(msg, "Rotation around %c-axis [on]", "xyz"[offset - 2]);
		else if(item >= MENU_CAMERA_MOVE_X && item <= MENU_CAMERA_MOVE_Z)
		    sprintf(msg, "Moving along %c-axis [on]", "xyz"[offset - 5]);
		else if(item == MENU_CAMERA_PROJECTION)
		    sprintf(msg, "Perspective[on]/orthogonal[off]");
	    }
	    ChangeToMenuEntry(offset + 1, msg, item);
	    SetMenu(m_idMenuMain);
	    return true;
	}
	else if(item >= MENU_DRAW_TEXTURE && item <= MENU_DRAW_EXPORT_FRAMES)
	{
	    const int  offset = item - MENU_DRAW_TEXTURE;
	    const int  val    = 1 << offset;
	    char  msg[50];

	    SetMenu(m_idMenuDraw);
	    if(m_gDrawAction & val)
	    {
		m_gDrawAction &=~val;
		if(item == MENU_DRAW_TEXTURE)
		    sprintf(msg, "Textures [off]");
		else if(item == MENU_DRAW_LIGHTING)
		    sprintf(msg, "Lighting [off]");
		else if(item == MENU_DRAW_WIREFRAME)
		    sprintf(msg, "Wireframe [off]");
		else if(item == MENU_DRAW_EXPORT_FRAMES)
		    sprintf(msg, "Export frames [off]");
	    }
	    else
	    {
		m_gDrawAction |= val;
		if(item == MENU_DRAW_TEXTURE)
		    sprintf(msg, "Textures [on]");
		else if(item == MENU_DRAW_LIGHTING)
		    sprintf(msg, "Lighting [on]");
		else if(item == MENU_DRAW_WIREFRAME)
		    sprintf(msg, "Wireframe [on]");
		else if(item == MENU_DRAW_EXPORT_FRAMES)
		    sprintf(msg, "Export frames [on]");
	    }
	    ChangeToMenuEntry(offset + 1, msg, item);
	    SetMenu(m_idMenuMain);
	    return true;
	}
	else if(item == MENU_DRAW_XY_BBOX)
	{
	    double xmin, ymin, xmax, ymax;

	    printf("Current xy bbox: %f %f %f %f\n",
		   GDrawGetParam(GDRAW_MINX),
		   GDrawGetParam(GDRAW_MINY),
		   GDrawGetParam(GDRAW_MAXX),
		   GDrawGetParam(GDRAW_MAXY));
	    printf("Enter new values:");
	    if(scanf("%lf %lf %lf %lf", &xmin, &ymin, &xmax, &ymax) != 4)
		OnInputError(printf("expecting xmin ymin xmax ymax\n"));
	    GDrawSetParam(GDRAW_MINX, xmin);
	    GDrawSetParam(GDRAW_MINY, ymin);
	    GDrawSetParam(GDRAW_MAXX, xmax);
	    GDrawSetParam(GDRAW_MAXY, ymax);
	    return true;
	}


	return false;
    }

    bool GManager::HandleEventOnNormalKeyPress(const int key)
    {
	if(key == 27) //escape key
	{
	    exit(0);
	    return true;
	}
	else if(glutGetModifiers() & GLUT_ACTIVE_ALT)
	{
	    if(key == '1')
	   	return HandleEventOnMenu(MENU_CAMERA_ROTATE_X);
	    else if(key == '2')
	   	return HandleEventOnMenu(MENU_CAMERA_ROTATE_Y);
	    else if(key == '3')
	   	return HandleEventOnMenu(MENU_CAMERA_ROTATE_Z);
	    else if(key == 'c')
		return HandleEventOnMenu(MENU_CAMERA_ROTATE_CENTER);
	    else if(key == 'g')
		return HandleEventOnMenu(MENU_CAMERA_ROTATE_GLOBAL_AXIS);
	    else if(key == 'p')
		return HandleEventOnMenu(MENU_CAMERA_PROJECTION);
	    else if(key == 't')
		return HandleEventOnMenu(MENU_DRAW_TEXTURE);
	    else if(key == 'l')
		return HandleEventOnMenu(MENU_DRAW_LIGHTING);
	    else if(key == 'w')
		return HandleEventOnMenu(MENU_DRAW_WIREFRAME);
	    else if(key == 'f')
	    {
		ExportFrameAsImage();

		return HandleEventOnMenu(MENU_DRAW_EXPORT_FRAMES);
	    }
      else if(key == 'v')
      {
    ExportFrameAsImage();
    }

	    else if(key == 'q')
	    {
		return HandleEventOnMenu(MENU_DRAW_EXPORT_FRAMES);
	    }


	    else if(key == 'x' || key == 'X' || key == 'y' || key == 'Y' || key == 'z' || key == 'Z')
	    {
		const double *p =
		    m_gCameraAction & ACTION_CAMERA_ROTATE_CENTER ?
		    m_gCamera.GetCenter() : m_gCamera.GetEye();
		const double theta = (key == 'x' || key == 'y' || key == 'z') ?
		    Constants::DEG2RAD :
		    -Constants::DEG2RAD;


		if(key == 'x' || key == 'X')
		{
		    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
			m_gCamera.RotateAroundAxisAtPoint(theta, 1, 0, 0, p[0], p[1], p[2]);
		    else
			m_gCamera.RotateAroundRightAxisAtPoint(theta, p[0], p[1], p[2]);
		}
		else if(key == 'y' || key == 'Y')
		{
		    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
			m_gCamera.RotateAroundAxisAtPoint(theta, 0, 1, 0, p[0], p[1], p[2]);
		    else
			m_gCamera.RotateAroundUpAxisAtPoint(theta, p[0], p[1], p[2]);
		}
		else if(key == 'z' || key == 'Z')
		{
		    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
			m_gCamera.RotateAroundAxisAtPoint(theta, 0, 0, 1, p[0], p[1], p[2]);
		    else
			m_gCamera.RotateAroundForwardAxisAtPoint(theta, p[0], p[1], p[2]);
		}
	    }
	    else if(key == 'e')
	    {

		const double *eye    = m_gCamera.GetEye();
		const double *center = m_gCamera.GetCenter();
		const double *right  = m_gCamera.GetRightAxis();
		const double *forward= m_gCamera.GetForwardAxis();

		printf("const double eye[] = {%f, %f, %f};\n", eye[0], eye[1], eye[2]);
		printf("const double center[] = {%f, %f, %f};\n", center[0], center[1], center[2]);
		printf("const double right[] = {%f, %f, %f};\n", right[0], right[1], right[2]);
		printf("const double forward[] = {%f, %f, %f};\n", forward[0], forward[1], forward[2]);
	    }


	}

	return false;
    }

    void GManager::Help(void)
    {
	BeginPrint(PRINT_INFO);
	printf("Help -- Menu options\n");
	printf(" right click to see menu\n\n");
	printf("Help -- Keyboard controls\n");
	printf(" F1         => help\n");
	printf(" Esc        => exit\n\n");
	printf(" ArrowUp    => move camera forward\n");
	printf(" ArrowDown  => move camera backward\n");
	printf(" ArrowLeft  => move camera left\n");
	printf(" ArrowRight => move camera right\n");
	printf(" PageUp     => move camera up\n");
	printf(" PageDown   => move camera down\n\n");
	printf(" Alt+x      => toggle rotate camera around x-axis [current = %s]\n",
	       (m_gCameraAction & ACTION_CAMERA_ROTATE_X) ? "on":"off");
	printf(" Alt+y      => toggle rotate camera around y-axis [current = %s]\n",
	       (m_gCameraAction & ACTION_CAMERA_ROTATE_Y) ? "on":"off");
	printf(" Alt+z      => toggle rotate camera around z-axis [current = %s]\n\n",
	       (m_gCameraAction & ACTION_CAMERA_ROTATE_Z) ? "on":"off");
	printf(" Alt+c      => toggle rotate camera about center/eye [current = %s]\n",
	       (m_gCameraAction & ACTION_CAMERA_ROTATE_CENTER) ? "center":"eye");
	printf(" Alt+g      => toggle rotate camera use global/own frame [current = %s]\n",
	       (m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS) ? "global":"own frame");
	printf(" Alt+p      => toggle perspective/orthogonal projection [current = %s]\n\n",
	       (m_gCameraAction & ACTION_CAMERA_PROJECTION) ? "perspective":"orthogonal");
	printf(" Alt+t      => toggle textures on/off  [current = %s]\n",
	       (m_gDrawAction & ACTION_DRAW_TEXTURES) ? "on":"off");
	printf(" Alt+l      => toggle lightining on/off [current = %s]\n",
	       (m_gDrawAction & ACTION_DRAW_LIGHTING) ? "on":"off");
	printf(" Alt+w      => toggle wireframe on/off [current = %s]\n",
	       (m_gDrawAction & ACTION_DRAW_WIREFRAME) ? "on":"off");
	printf(" Alt+f      => toggle export frames on/off [current = %s]\n",
	       (m_gDrawAction & ACTION_DRAW_EXPORT_FRAMES) ? "on":"off");

	EndPrint();

    }



    bool GManager::HandleEventOnSpecialKeyPress(const int key)
    {
	switch(key)
	{
	case GLUT_KEY_F1:
	    Help();
	    return true;

	case GLUT_KEY_UP:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_Y)
	    {
		if(m_gCameraAction & ACTION_CAMERA_PROJECTION)
		    m_gCamera.MoveForward(m_gCameraMove);
		else
		    m_viewZoom -= m_gCameraMove;
		return true;
	    }
	    break;

	case GLUT_KEY_DOWN:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_Y)
	    {
		if(m_gCameraAction & ACTION_CAMERA_PROJECTION)
		    m_gCamera.MoveForward(-m_gCameraMove);
		else
		    m_viewZoom += m_gCameraMove;
		return true;
	    }
	    break;


	case GLUT_KEY_RIGHT:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_X)
	    {
		m_gCamera.MoveRight(m_gCameraMove);
		return true;
	    }
	    break;


	case GLUT_KEY_LEFT:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_X)
	    {
		m_gCamera.MoveRight(-m_gCameraMove);
		return true;
	    }
	    break;

	case GLUT_KEY_PAGE_DOWN:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_Z)
	    {
		m_gCamera.MoveUp(m_gCameraMove);
		return true;
	    }
	    break;

	case GLUT_KEY_PAGE_UP:
	    if(m_gCameraAction & ACTION_CAMERA_MOVE_Z)
	    {
		m_gCamera.MoveUp(-m_gCameraMove);
		return true;
	    }
	    break;
	}

	return false;
    }

    bool GManager::HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
	return false;
    }

    bool GManager::HandleEventOnActiveMouseMove(const int x, const int y)
    {
	const double thetax = 2 * M_PI * (y - m_mousePrevY) / glutGet(GLUT_WINDOW_HEIGHT);
	const double thetay = 2 * M_PI * (x - m_mousePrevX) / glutGet(GLUT_WINDOW_WIDTH);
	const double thetaz = thetay;
	bool         event  = false;

	const double *p =
	    m_gCameraAction & ACTION_CAMERA_ROTATE_CENTER ?
	    m_gCamera.GetCenter() : m_gCamera.GetEye();


	if(m_gCameraAction & ACTION_CAMERA_ROTATE_X)
	{
	    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
		m_gCamera.RotateAroundAxisAtPoint(thetax, 1, 0, 0, p[0], p[1], p[2]);
	    else
		m_gCamera.RotateAroundRightAxisAtPoint(thetax, p[0], p[1], p[2]);
	    event = true;
	}

	if(m_gCameraAction & ACTION_CAMERA_ROTATE_Y)
	{
	    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
		m_gCamera.RotateAroundAxisAtPoint(thetay, 0, 1, 0, p[0], p[1], p[2]);
	    else
		m_gCamera.RotateAroundUpAxisAtPoint(thetay, p[0], p[1], p[2]);
	    event = true;
	}

	if(m_gCameraAction & ACTION_CAMERA_ROTATE_Z)
	{
	    if(m_gCameraAction & ACTION_CAMERA_ROTATE_GLOBAL_AXIS)
		m_gCamera.RotateAroundAxisAtPoint(thetaz, 0, 0, 1, p[0], p[1], p[2]);
	    else
		m_gCamera.RotateAroundForwardAxisAtPoint(thetaz, p[0], p[1], p[2]);
	    event = true;
	}
	return event;
    }

    bool GManager::HandleEventOnPassiveMouseMove(const int x, const int y)
    {
	return false;
    }

    bool GManager::HandleEventOnIdle(void)
    {
	return false;
    }


    void GManager::MousePosFromScreenToWorld(const int x, const int y,
					     double * const wx,
					     double * const wy,
					     double * const wz)
    {
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ=0;
	double px = 0, py = 0, pz = 0;

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &px, &py, &pz);

	if(wx)
	    *wx = px;
	if(wy)
	    *wy = py;
	if(wz)
	    *wz = pz;
    }

    void GManager::ExportFrameAsImage(void)
    {
	int  ignore;
	char fname[50];
	sprintf(fname, "frames_%05d.ppm", (m_gManager->m_frames)++);
	ExportFrameAsImage(fname);

	char cmd[300];
	sprintf(cmd, "convert -quality 100 %s %s.jpg", fname, fname);
	ignore = system(cmd);
	sprintf(cmd, "\\rm %s", fname);
	ignore = system(cmd);
    }

    void GManager::ExportFrameAsImage(const char fname[])
    {

	const int width = glutGet(GLUT_WINDOW_WIDTH);
	const int height= glutGet(GLUT_WINDOW_HEIGHT);

	char *temp  = new char[3 * width * height];
	char *image = new char[3 * width * height];

	FILE *fp = fopen(fname, "w");

	printf("Writing %s\n", fname);

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, temp);

	int  a, b, row_sz = 3*width;
	// Reverse rows
	for(int i=0; i < height; i+=1)
	{
	    for(int j=0; j < width; j+=1)
	    {
		a = i*row_sz+3*j;
		b = (height-i-1)*row_sz+3*j;
		image[a]   = temp[b];
		image[a+1] = temp[b+1];
		image[a+2] = temp[b+2];
	    }
	}
	fprintf(fp, "P6\n");
	fprintf(fp, "%i %i\n 255\n", width, height);
	fwrite(image, sizeof(char), 3 * width * height, fp);
	fclose(fp);
	delete[] temp;
	delete[] image;
    }

    int GManager::CreateMenu(void)
    {
	return glutCreateMenu(CallbackEventOnMenu);
    }

    void GManager::SetMenu(const int menu)
    {
	glutSetMenu(menu);
    }

    void GManager::AddSubMenu(const char name[], const int menu)
    {
	glutAddSubMenu(name, menu);
    }

    void GManager::AddMenuEntry(const char name[], const int item)
    {
	glutAddMenuEntry(name, item);
    }

    void GManager::ChangeToMenuEntry(const int pos, const char name[], const int item)
    {
	glutChangeToMenuEntry(pos, name, item);
    }
}
