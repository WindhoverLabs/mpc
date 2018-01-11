/**
 * @file Dcm.cpp
 *
 * A direction cosine matrix class.
 * All rotations and axis systems follow the right-hand rule.
 *
 * This library uses the convention that premultiplying a three dimensional
 * vector represented in coordinate system 1 will apply a rotation from coordinate system
 * 1 to coordinate system 2 to the vector.
 * Likewise, a matrix instance of this class also represents a coordinate transformation
 * from frame 2 to frame 1.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "DcmF.hpp"

namespace math
{

/**
 * Direction cosine matrix class
 *
 * The rotation between two coordinate frames is
 * described by this class.
 */
class DcmF : public Matrix3F3
{
public:
    floatdef Matrix<float, 3, 1> Vector3;

    /**
     * Standard constructor
     *
     * Initializes to identity
     */
    Dcm() : SquareMatrix<float, 3>()
    {
        (*this) = eye<float, 3>();
    }

    /**
     * Constructor from array
     *
     * @param _data pointer to array
     */
    Dcm(const float *data_) : SquareMatrix<float, 3>(data_)
    {
    }

    /**
     * Copy constructor
     *
     * @param other Matrix33 to set dcm to
     */
    Dcm(const Matrix<float, 3, 3> &other) : SquareMatrix<float, 3>(other)
    {
    }

    /**
     * Constructor from quaternion
     *
     * Instance is initialized from quaternion representing
     * coordinate transformation from frame 2 to frame 1.
     *
     * @param q quaternion to set dcm to
     */
    Dcm(const Quaternion<float> &q)
    {
        Dcm &dcm = *this;
        float a = q(0);
        float b = q(1);
        float c = q(2);
        float d = q(3);
        float aSq = a * a;
        float bSq = b * b;
        float cSq = c * c;
        float dSq = d * d;
        dcm(0, 0) = aSq + bSq - cSq - dSq;
        dcm(0, 1) = 2 * (b * c - a * d);
        dcm(0, 2) = 2 * (a * c + b * d);
        dcm(1, 0) = 2 * (b * c + a * d);
        dcm(1, 1) = aSq - bSq + cSq - dSq;
        dcm(1, 2) = 2 * (c * d - a * b);
        dcm(2, 0) = 2 * (b * d - a * c);
        dcm(2, 1) = 2 * (a * b + c * d);
        dcm(2, 2) = aSq - bSq - cSq + dSq;
    }

    /**
     * Constructor from euler angles
     *
     * This sets the transformation matrix from frame 2 to frame 1 where the rotation
     * from frame 1 to frame 2 is described by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     *
     * @param euler euler angle instance
     */
    Dcm(const Euler<float> &euler)
    {
        Dcm &dcm = *this;
        float cosPhi = float(cos(euler.phi()));
        float sinPhi = float(sin(euler.phi()));
        float cosThe = float(cos(euler.theta()));
        float sinThe = float(sin(euler.theta()));
        float cosPsi = float(cos(euler.psi()));
        float sinPsi = float(sin(euler.psi()));

        dcm(0, 0) = cosThe * cosPsi;
        dcm(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
        dcm(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

        dcm(1, 0) = cosThe * sinPsi;
        dcm(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
        dcm(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

        dcm(2, 0) = -sinThe;
        dcm(2, 1) = sinPhi * cosThe;
        dcm(2, 2) = cosPhi * cosThe;
    }


    /**
     * Constructor from axis angle
     *
     * This sets the transformation matrix from frame 2 to frame 1 where the rotation
     * from frame 1 to frame 2 is described by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     *
     * @param euler euler angle instance
     */
    Dcm(const AxisAngle<float> &aa)
    {
        Dcm &dcm = *this;
        dcm = Quaternion<float>(aa);
    }

    Vector<float, 3> vee() const      // inverse to Vector.hat() operation
    {
        const Dcm &A(*this);
        Vector<float, 3> v;
        v(0) = -A(1, 2);
        v(1) =  A(0, 2);
        v(2) = -A(0, 1);
        return v;
    }

    void renormalize()
    {
        /* renormalize rows */
        for (size_t row = 0; row < 3; row++) {
            matrix::Vector3f rvec(this->_data[row]);
            this->setRow(row, rvec.normalized());
        }
    }
};

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
