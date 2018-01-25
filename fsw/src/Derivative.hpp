#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include "cfe.h"
#include "LowPassFilter2p.h"

class Derivative
{
public:
	Derivative();
	~Derivative();
	float Update(float value, float dt, float cutoff);

private:
	bool m_Initialized;
	float m_PreviousValue;
    math::LowPassFilter2p m_LowPass;
};

#endif
