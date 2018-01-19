#ifndef MP_GMATERIAL_HPP_
#define MP_GMATERIAL_HPP_

#include <cstdio>
#include "Utils/PrintMsg.hpp"

namespace MP
{
    class GMaterial
    {
    public:
	GMaterial(void)
	{
//	    SetPearl();
//	    SetRuby();
	    SetObsidian();


	}

	virtual ~GMaterial(void)
	{
	}

	enum Property
	    {
		AMBIENT     = 0,
		DIFFUSE     = 1,
		SPECULAR    = 2,
		EMMISSIVE   = 3,
		SHININESS   = 4
	    };

	virtual const float* GetProperty(const Property type) const
	{
	    return m_prop[type];
	}

	virtual void SetProperty(const Property type, const float * const prop)
	{
	    SetProperty(type, prop[0], prop[1], prop[2], prop[3]);
	}

	virtual void SetProperty(const Property type,
				 const float a = 0.0f,
				 const float b = 0.0f,
				 const float c = 0.0f,
				 const float d = 1.0f)
	{
	    m_prop[type][0] = a;
	    m_prop[type][1] = b;
	    m_prop[type][2] = c;
	    m_prop[type][3] = d;
	}

	void SetAmbient(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(AMBIENT, a, b, c, d);
	}
	void SetDiffuse(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(DIFFUSE, a, b, c, d);
	}
	void SetSpecular(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(SPECULAR, a, b, c, d);
	}
	void SetEmmissive(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(EMMISSIVE, a, b, c, d);
	}
	void SetShininess(const float a)
	{
	    SetProperty(SHININESS, a);
	}

	void CopyFrom(const GMaterial * const gMat)
	{
	    SetProperty(AMBIENT,   gMat->GetProperty(AMBIENT));
	    SetProperty(DIFFUSE,   gMat->GetProperty(DIFFUSE));
	    SetProperty(SPECULAR,  gMat->GetProperty(SPECULAR));
	    SetProperty(EMMISSIVE, gMat->GetProperty(EMMISSIVE));
	    SetProperty(SHININESS, gMat->GetProperty(SHININESS));
	}


	void SetGold(void)
	{
	    SetAmbient(0.24725, 	0.1995, 	0.0745);
	    SetDiffuse(0.75164, 	0.60648, 	0.22648);
	    SetShininess(20.4);
	    SetEmmissive(0, 0, 0, 1);
	    SetSpecular(1, 1, 1, 1);


	}

	void SetPearl(void)
	{
	    SetAmbient(0.25, 	0.20725, 	0.20725);
	    SetDiffuse(1, 	0.829, 	0.829);
	    SetShininess(18.088);
	    SetEmmissive(0, 0, 0, 1);
	    SetSpecular(1, 1, 1, 1);


	}

	void SetRuby(void)
	{
	    SetAmbient(0.1745,0.01175,0.01175);
	    SetDiffuse(0.61424,0.04136,0.04136);
	    SetSpecular(0.727811,0.626959,0.626959);
	    SetShininess(0.6*128);

	}

	void SetObsidian(void)
	{
	    SetAmbient(0.05375,0.05,0.06625);
	    SetSpecular(0.332741,0.328634,0.346435);
	    SetDiffuse(0.18275,0.17,0.22525);
	    SetShininess(0.6*128);
	}

	void SetTurquoise(void)
	{
	    SetAmbient(0.1, 0.18725,0.1745);
	    SetDiffuse(0.396,0.74151,0.69102);
	    SetSpecular(0.297254,0.30829,0.306678);
	    SetShininess(0.1*128);
	}

	void SetYellowPlastic(void)
	{
	    SetAmbient(0,0,0);
	    SetDiffuse(1,1,0.0);
	    SetSpecular(0.60,0.60,0.50);
	    SetShininess(10.25*128);
	}

	void Read(FILE * const in)
	{
	    float a, b, c, d;
	    for(int i = 0; i < 5; ++i)
	    {
		if(fscanf(in, "%f %f %f %f", &a, &b, &c, &d) != 4)
		    OnInputError(printf("expecting 4 values for property %d/%d\n", i, 5));
		SetProperty((Property) i, a, b, c, d);
	    }
	}

	void Print(FILE * const out) const
	{
	    for(int i = 0; i < 5; ++i)
		fprintf(out, "%f %f %f %f\n", m_prop[i][0], m_prop[i][1], m_prop[i][2], m_prop[i][3]);

	}



    protected:
	float m_prop[5][4];


    };
}


#endif
