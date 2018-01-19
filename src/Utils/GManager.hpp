#ifndef  MP_GMANAGER_HPP_
#define  MP_GMANAGER_HPP_

#include "Utils/GDraw.hpp"
#include "Utils/GCamera.hpp"


namespace MP
{
    class GManager
    {
    public:
	GManager(void);
	
	virtual ~GManager(void);
	
	virtual void Help(void);
	
	virtual void MainLoop(const char * const title, 
			      const int width = 1000, 
			      const int height=  600);
	
	static void MousePosFromScreenToWorld(const int x, const int y, 
					      double * const wx,
					      double * const wy,
					      double * const wz);
	
	static void ExportFrameAsImage(void);
	
	static void ExportFrameAsImage(const char fname[]);
	
	
	
	virtual void MenuOptions(void);
	
	virtual void HandleEventOnTimer(void)
	{
	}
	
	
	virtual void Defaults2D(void)
	{
	    m_gCameraAction &= ~ACTION_CAMERA_PROJECTION;
	    m_gCameraAction &= ~ACTION_CAMERA_ROTATE_X;
	    m_gCameraAction &= ~ACTION_CAMERA_ROTATE_Y;
	    m_gCameraAction &= ~ACTION_CAMERA_ROTATE_Z;
	    
	    m_gDrawAction   &= ~ACTION_DRAW_LIGHTING;
	    m_gDrawAction   &= ~ACTION_DRAW_TEXTURES;
	}
	
	virtual void HandleEventOnDisplay(void)
	{
	}
	
	virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y);
	virtual bool HandleEventOnActiveMouseMove(const int x, const int y);
	virtual bool HandleEventOnPassiveMouseMove(const int x, const int y);
	virtual bool HandleEventOnNormalKeyPress(const int key);
	virtual bool HandleEventOnSpecialKeyPress(const int key);
	virtual bool HandleEventOnNormalKeyUp(const int key)
	{
	    return false;
	}
	
	virtual bool HandleEventOnSpecialKeyUp(const int key)
	{
	    return false;
	}
	
	virtual bool HandleEventOnMenu(const int item);
	
	virtual bool HandleEventOnIdle(void);
	

    protected:	
	static void CallbackEventOnActiveMouseMove(int x, int y);
	static void CallbackEventOnPassiveMouseMove(int x, int y);
	static void CallbackEventOnMouse(int button, int state, int x, int y);
	static void CallbackEventOnTimer(int id);
	static void CallbackEventOnMenu(int item);
	static void CallbackEventOnSpecialKeyPress(int key, int x, int y);
	static void CallbackEventOnNormalKeyPress(unsigned char key, int x, int y);
	static void CallbackEventOnSpecialKeyUp(int key, int x, int y);
	static void CallbackEventOnNormalKeyUp(unsigned char key, int x, int y);

	static void CallbackEventOnDisplay(void);
	static void CallbackEventOnIdle(void);
	
	
	
	static int  CreateMenu(void);
	static void SetMenu(const int menu);	    
	static void AddSubMenu(const char name[], const int item);	    
	static void AddMenuEntry(const char name[], const int item);	   
	static void ChangeToMenuEntry(const int pos, const char name[], const int item);
	
	
	
	int      m_idWindow;
	int      m_timer;
	GCamera  m_gCamera;
	
	int m_nrMenuItems;	    
	int m_idMenuMain;
	int m_idMenuDraw;
	int MENU_DRAW_TEXTURE;
	int MENU_DRAW_LIGHTING;
	int MENU_DRAW_EXPORT_FRAMES;	    
	int MENU_DRAW_WIREFRAME;	    
	int MENU_DRAW_XY_BBOX;	    
	int m_gDrawAction;	    
	int m_idMenuCamera;
	int MENU_CAMERA_ROTATE_CENTER;	    
	int MENU_CAMERA_ROTATE_GLOBAL_AXIS;	    
	int MENU_CAMERA_ROTATE_X;
	int MENU_CAMERA_ROTATE_Y;
	int MENU_CAMERA_ROTATE_Z;
	int MENU_CAMERA_MOVE_X;
	int MENU_CAMERA_MOVE_Y;
	int MENU_CAMERA_MOVE_Z;
	int MENU_CAMERA_PROJECTION;
	int MENU_CAMERA_STANDARD;
	int m_gCameraAction;
	double m_gCameraMove;
	
	int m_mousePrevX;
	int m_mousePrevY;
	int m_frames;
	
	double m_viewZoom;
	
	enum
	    {
		ACTION_CAMERA_ROTATE_CENTER  =   1,
		ACTION_CAMERA_ROTATE_GLOBAL_AXIS = 2,
		ACTION_CAMERA_ROTATE_X       =   4,
		ACTION_CAMERA_ROTATE_Y       =   8,
		ACTION_CAMERA_ROTATE_Z       =  16,
		ACTION_CAMERA_MOVE_X         =  32,
		ACTION_CAMERA_MOVE_Y         =  64,
		ACTION_CAMERA_MOVE_Z         = 128,
		ACTION_CAMERA_PROJECTION     = 256,
		
		ACTION_DRAW_TEXTURES         = 1,
		ACTION_DRAW_LIGHTING         = 2,
		ACTION_DRAW_WIREFRAME        = 4,
		ACTION_DRAW_EXPORT_FRAMES    = 8,
	    };
	
    };
}

#endif
