#ifndef VECTOR2F_HPP
#define VECTOR2F_HPP

#include "cfe.h"

namespace math
{

class Vector2F
{
private:
	static const int SIZE = 2;
	float data[SIZE];
	float nan;

public:
	Vector2F();
	Vector2F(float X, float Y);
	~Vector2F();
	float& operator [] (uint32 i);
	float operator [] (uint32 i) const;
	float Length(void) const;
	void Zero(void);
	Vector2F operator+(const Vector2F &vecIn);
	Vector2F operator-(const Vector2F &vecIn);
	Vector2F operator%(const Vector2F &vecIn);
	Vector2F operator*(const float scalar);
	Vector2F operator/(const float scalar);
	float operator*(const Vector2F &vecIn);
	Vector2F operator-() const;
	Vector2F EMult(const Vector2F &vecIn);
	void Constrain(uint32 i, float min, float max);
	Vector2F Normalized(void);

protected:

};

}

#endif
