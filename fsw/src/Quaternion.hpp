#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "Vector4F.hpp"
#include "Vector3F.hpp"
#include "Matrix3F3.hpp"

namespace math
{

class Quaternion : public Vector4F
{
public:

public:
	Quaternion(float a, float b, float c, float d);
	Quaternion(const Matrix3F3 &mat);
	~Quaternion();
	Matrix3F3 RotationMatrix(void) const;
    Vector3F Imaginary(void);
    Vector3F ToEuler(void) const;

protected:
	Quaternion();
};

}

#endif
