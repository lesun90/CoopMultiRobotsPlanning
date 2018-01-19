#ifndef MP_GTEXTURE_HPP_
#define MP_GTEXTURE_HPP_

#include <cstdio>

namespace MP
{
    class GTexture
    {
    public:
	GTexture(void);	

	virtual ~GTexture(void);

	const char* GetFileName(void) const
	{
	    return m_fname;
	}
	
	
	unsigned int GetId(void) const
	{
	    return m_id;
	}
	
	unsigned char* GetImage(void) const
	{
	    return m_image;
	}
	void SetFileName(const char fname[]);
	void ReadBMP(FILE *in);
	void ReadPPM(FILE *in);
	void ReadPNG(FILE *in);


	static void ManualCoords(void);	
	static void AutomaticCoords(void);
	
	void Use(void);
	
    protected:    
	void ToRGBA(void);
	
	unsigned int   m_id;
	unsigned char *m_image;	
	int            m_sizeX;
	int            m_sizeY;
	char          *m_fname;
	bool           m_rgba;	
    };
}

#endif









