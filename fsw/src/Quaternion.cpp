#include "Quaternion.hpp"
#include <math.h>

using namespace math;

Quaternion::Quaternion(float a, float b, float c, float d) :
	Vector4F(a, b, c, d)
{

}


Quaternion::Quaternion(const Matrix3F3 &mat)
{
	float tr = mat[0][0] + mat[1][1] + mat[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		data[0] = s * 0.5f;
		s = 0.5f / s;
		data[1] = (mat[2][1] - mat[1][2]) * s;
		data[2] = (mat[0][2] - mat[2][0]) * s;
		data[3] = (mat[1][0] - mat[0][1]) * s;
	} else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int mat_i = 0;
		for (int i = 1; i < 3; i++) {
			if (mat[i][i] > mat[mat_i][mat_i]) {
				mat_i = i;
			}
		}
		int mat_j = (mat_i + 1) % 3;
		int mat_k = (mat_i + 2) % 3;
		float s = sqrtf((mat[mat_i][mat_i] - mat[mat_j][mat_j] -
				mat[mat_k][mat_k]) + 1.0f);
		data[mat_i + 1] = s * 0.5f;
		s = 0.5f / s;
		data[mat_j + 1] = (mat[mat_i][mat_j] + mat[mat_j][mat_i]) * s;
		data[mat_k + 1] = (mat[mat_k][mat_i] + mat[mat_i][mat_k]) * s;
		data[0] = (mat[mat_k][mat_j] - mat[mat_j][mat_k]) * s;
	}
}


Quaternion::Quaternion()
{

}


Quaternion::~Quaternion()
{

}


Matrix3F3 Quaternion::RotationMatrix(void) const
{
	Matrix3F3 R;
	float aSq = data[0] * data[0];
	float bSq = data[1] * data[1];
	float cSq = data[2] * data[2];
	float dSq = data[3] * data[3];
	R[0][0] = aSq + bSq - cSq - dSq;
	R[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
	R[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
	R[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
	R[1][1] = aSq - bSq + cSq - dSq;
	R[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
	R[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
	R[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
	R[2][2] = aSq - bSq - cSq + dSq;
	return R;
}


Vector3F Quaternion::Imaginary(void)
{
    return Vector3F(data[1], data[2], data[3]);
}



Vector3F Quaternion::ToEuler(void) const
{
	Vector3F euler(
		atan2f(2.0f * (data[0] * data[1] + data[2] * data[3]), 1.0f - 2.0f * (data[1] * data[1] + data[2] * data[2])),
		asinf(2.0f * (data[0] * data[2] - data[3] * data[1])),
		atan2f(2.0f * (data[0] * data[3] + data[1] * data[2]), 1.0f - 2.0f * (data[2] * data[2] + data[3] * data[3]))
	);

	return euler;
}
