#ifndef MP_CREATE_SCENE_HPP_
#define MP_CREATE_SCENE_HPP_

#include "Scenes/Scene.hpp"

namespace MP
{
    class CreateScene
    {
    public:
	CreateScene(void)
	{
	}

	virtual ~CreateScene(void)
	{
	}

	static double DefaultThickness(void)
	{
	    return 1.0;
	}

	static double DefaultZMax(void)
	{
	    return 9.0;
	}

	static void DefaultGrid(Grid * grid)
	{
	    grid->Setup2D(32, 32, -60, -60, 60, 60);
	}

	static void DefaultBoundaries(Scene * const scene, const double z = 9)
	{
	    scene->m_tmesh.AddBoundaries(-60, -60, 0,
					 60,   60, z, DefaultThickness());
	}

	static void Print(Scene * const scene, const char prefix[]);

    };

}

#endif
