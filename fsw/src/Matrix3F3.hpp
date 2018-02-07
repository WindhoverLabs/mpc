#ifndef MATRIX3F3_HPP
#define MATRIX3F3_HPP

#include "cfe.h"
#include "Matrix3F3.hpp"
#include "Vector3F.hpp"

#define M_DEG_TO_RAD_F		(0.01745329251994329576f)

namespace math
{

class Matrix3F3
{
public:
	typedef enum {
		ROTATION_NONE                = 0,
		ROTATION_YAW_45              = 1,
		ROTATION_YAW_90              = 2,
		ROTATION_YAW_135             = 3,
		ROTATION_YAW_180             = 4,
		ROTATION_YAW_225             = 5,
		ROTATION_YAW_270             = 6,
		ROTATION_YAW_315             = 7,
		ROTATION_ROLL_180            = 8,
		ROTATION_ROLL_180_YAW_45     = 9,
		ROTATION_ROLL_180_YAW_90     = 10,
		ROTATION_ROLL_180_YAW_135    = 11,
		ROTATION_PITCH_180           = 12,
		ROTATION_ROLL_180_YAW_225    = 13,
		ROTATION_ROLL_180_YAW_270    = 14,
		ROTATION_ROLL_180_YAW_315    = 15,
		ROTATION_ROLL_90             = 16,
		ROTATION_ROLL_90_YAW_45      = 17,
		ROTATION_ROLL_90_YAW_90      = 18,
		ROTATION_ROLL_90_YAW_135     = 19,
		ROTATION_ROLL_270            = 20,
		ROTATION_ROLL_270_YAW_45     = 21,
		ROTATION_ROLL_270_YAW_90     = 22,
		ROTATION_ROLL_270_YAW_135    = 23,
		ROTATION_PITCH_90            = 24,
		ROTATION_PITCH_270           = 25,
		ROTATION_ROLL_270_YAW_270    = 26,
		ROTATION_ROLL_180_PITCH_270  = 27,
		ROTATION_PITCH_90_YAW_180    = 28,
		ROTATION_PITCH_90_ROLL_90	 = 29,
		ROTATION_YAW_293_PITCH_68_ROLL_90 = 30,
		ROTATION_PITCH_90_ROLL_270	 = 31,
		ROTATION_PITCH_9_YAW_180 = 32,
		ROTATION_MAX
	} Rotation_t;

	typedef struct {
		uint16 roll;
		uint16 pitch;
		uint16 yaw;
	} RotLookup_t;

private:
	static const int SIZE = 3;
	Vector3F data[SIZE];
	Vector3F nan;

	static const RotLookup_t RotLookup[];

public:
	Matrix3F3(Vector3F m0, Vector3F m1, Vector3F m2);
	~Matrix3F3();
	Vector3F& operator [] (uint32 i);
	Vector3F operator [] (uint32 i) const;
	Matrix3F3();
	Matrix3F3 Transpose(void);
	static Matrix3F3 Identity(void);
	Vector3F operator*(const Vector3F &vecIn);
	Matrix3F3 operator*(const Matrix3F3 &matIn);
	Matrix3F3 operator*(const float &scalar);
	void Zero(void);
    Matrix3F3 operator+(const Matrix3F3 &matIn) const;
	static Matrix3F3 RotationMatrix(Matrix3F3::Rotation_t boardRotation);
	static Matrix3F3 FromEuler(float roll, float pitch, float yaw);
	Vector3F ToEuler(void) const;

protected:

};

}

#endif
