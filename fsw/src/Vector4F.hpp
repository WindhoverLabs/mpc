#ifndef VECTOR4F_HPP
#define VECTOR4F_HPP

#include "cfe.h"

namespace math
{

class Vector4F
{
private:
	static const int SIZE = 4;
	float nan;

public:
	Vector4F(float m1, float m2, float m3, float m4);
	~Vector4F();
	float& operator [] (uint32 i);
	float operator [] (uint32 i) const;
	Vector4F operator-() const;

protected:
	Vector4F();
	float data[SIZE];

};

}

#endif
