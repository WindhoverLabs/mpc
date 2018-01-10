#include "Vector4F.hpp"
#include <math.h>

using namespace math;

Vector4F::Vector4F(float m1, float m2, float m3, float m4) :
    data{m1, m2, m3, m4},
	nan(NAN)
{
};


Vector4F::Vector4F()
{
};


Vector4F::~Vector4F()
{
};


float& Vector4F::operator [] (uint32 i)
{
	if(i < SIZE)
	{
		return data[i];
	}

	return nan;
};


float Vector4F::operator [] (uint32 i) const
{
	if(i < SIZE)
	{
		return data[i];
	}

	return NAN;
};


Vector4F Vector4F::operator-() const
{
	Vector4F res;

	res[0] = -data[0];
	res[1] = -data[1];
	res[2] = -data[2];
	res[3] = -data[3];

    return res;
}
