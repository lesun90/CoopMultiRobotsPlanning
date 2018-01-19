#ifndef MP_PID_CONTROLLER_HPP_
#define MP_PID_CONTROLLER_HPP_

#include <cstdlib>

namespace MP
{
    class PIDController
    {
    public:
	PIDController(void)
	{
	    Kp = 10;
	    Ki = 0.1;
	    Kd = 0.1;

	    m_desiredValue = 0;
	    m_errorIntegral = m_errorPrevious = 0;
	}

	virtual void SetDesiredValue(const double val)
	{
	    m_desiredValue = val;
	}

	virtual void Reset(void)
	{
	    m_errorIntegral = m_errorPrevious = 0;
	}

	virtual double Update(const double currValue, const double dt)
	{
	    const double error = m_desiredValue - currValue;
	    m_errorIntegral += error * dt;
	    const double u = Kp * error + Ki * m_errorIntegral +
		             Kd * (error - m_errorPrevious) / dt;
	    m_errorPrevious = error;

	    return u;
	}

	double Kp;
	double Ki;
	double Kd;

//    protected:

	double m_desiredValue;
	double m_errorIntegral;
	double m_errorPrevious;
    };
}


#endif
