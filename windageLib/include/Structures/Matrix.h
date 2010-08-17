/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
 *   U-VR Lab, GIST of Gwangju in Korea.
 *   http://windage.googlecode.com/
 *   http://uvr.gist.ac.kr/
 *
 * Copyright of the derived and new portions of this work
 *     (C) 2009 GIST U-VR Lab.
 *
 * This framework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information please contact 
 *   Woonhyuk Baek
 *   <windage@live.com>
 *   GIST U-VR Lab.
 *   Department of Information and Communication
 *   Gwangju Institute of Science and Technology
 *   1, Oryong-dong, Buk-gu, Gwangju
 *   South Korea
 * ========================================================================
 ** @author   Woonhyuk Baek
 * ======================================================================== */

/**
 * @file	Matrix.h
 * @author	Woonhyuk Baek
 * @brief	2x2, 3x3, 4x4 Matrix structures
 */

#ifndef _WBAEK_MATRIX_H_
#define _WBAEK_MATRIX_H_

#include "memory.h"
#include "Vector.h"

namespace windage
{
	/**
	 * @defgroup Structures Data Structures
	 * @brief
	 *		data structures classes
	 * @addtogroup Structures
	 * @{
	 */

	/**
	 * @brief	Matrix4
	 * @author	Woonhyuk Baek
	 */
	struct Matrix4{
		union{
			struct{double m[4][4];};
			struct{double m1[16];};
			struct{double	_11, _12, _13, _14,
							_21, _22, _23, _24,
							_31, _32, _33, _34,
							_41, _42, _43, _44;
					};
		};

		Matrix4()
		{
			m[0][0] = 0;	m[0][1] = 0;	m[0][2] = 0;	m[0][3] = 0; 
			m[1][0] = 0;	m[1][1] = 0;	m[1][2] = 0;	m[1][3] = 0;
			m[2][0] = 0;	m[2][1] = 0;	m[2][2] = 0;	m[2][3] = 0;
			m[3][0] = 0;	m[3][1] = 0;	m[3][2] = 0;	m[3][3] = 0;
		}
		
		
		Matrix4(	double m11, double m12, double m13, double m14,
					double m21, double m22, double m23, double m24,
					double m31, double m32, double m33, double m34,
					double m41, double m42, double m43, double m44)
		{
			m[0][0] = m11;	m[0][1] = m12;	m[0][2] = m13;	m[0][3] = m14;
			m[1][0] = m21;	m[1][1] = m22;	m[1][2] = m23;	m[1][3] = m24;
			m[2][0] = m31;	m[2][1] = m32;	m[2][2] = m33;	m[2][3] = m34;
			m[3][0] = m41;	m[3][1] = m42;	m[3][2] = m43;	m[3][3] = m44;
		}

		Matrix4 operator+(const Matrix4 &rhs) const
		{
			return	Matrix4(m[0][0] + rhs.m[0][0],	m[0][1] + rhs.m[0][1],	m[0][2] + rhs.m[0][2], m[0][3] + rhs.m[0][3],
							m[1][0] + rhs.m[1][0],	m[1][1] + rhs.m[1][1],	m[1][2] + rhs.m[1][2], m[1][3] + rhs.m[1][3],
							m[2][0] + rhs.m[2][0],	m[2][1] + rhs.m[2][1],	m[2][2] + rhs.m[2][2], m[2][3] + rhs.m[2][3],
							m[3][0] + rhs.m[3][0],	m[3][1] + rhs.m[3][1],	m[3][2] + rhs.m[3][2], m[3][3] + rhs.m[3][3]);
		}

		Matrix4 operator-(const Matrix4 &rhs) const
		{
			return	Matrix4(m[0][0] - rhs.m[0][0],	m[0][1] - rhs.m[0][1],	m[0][2] - rhs.m[0][2], m[0][3] - rhs.m[0][3],
							m[1][0] - rhs.m[1][0],	m[1][1] - rhs.m[1][1],	m[1][2] - rhs.m[1][2], m[1][3] - rhs.m[1][3],
							m[2][0] - rhs.m[2][0],	m[2][1] - rhs.m[2][1],	m[2][2] - rhs.m[2][2], m[2][3] - rhs.m[2][3],
							m[3][0] - rhs.m[3][0],	m[3][1] - rhs.m[3][1],	m[3][2] - rhs.m[3][2], m[3][3] - rhs.m[3][3]);
		}

		Matrix4 operator*(const Matrix4 &rhs) const
		{
			return Matrix4(m[0][0] * rhs.m[0][0] + m[0][1] * rhs.m[1][0] + m[0][2] * rhs.m[2][0] + m[0][3] * rhs.m[3][0],
							m[0][0] * rhs.m[0][1] + m[0][1] * rhs.m[1][1] + m[0][2] * rhs.m[2][1] + m[0][3] * rhs.m[3][1],
							m[0][0] * rhs.m[0][2] + m[0][1] * rhs.m[1][2] + m[0][2] * rhs.m[2][2] + m[0][3] * rhs.m[3][2],
							m[0][0] * rhs.m[0][3] + m[0][1] * rhs.m[1][3] + m[0][2] * rhs.m[2][3] + m[0][3] * rhs.m[3][3],

							m[1][0] * rhs.m[0][0] + m[1][1] * rhs.m[1][0] + m[1][2] * rhs.m[2][0] + m[1][3] * rhs.m[3][0],
							m[1][0] * rhs.m[0][1] + m[1][1] * rhs.m[1][1] + m[1][2] * rhs.m[2][1] + m[1][3] * rhs.m[3][1],
							m[1][0] * rhs.m[0][2] + m[1][1] * rhs.m[1][2] + m[1][2] * rhs.m[2][2] + m[1][3] * rhs.m[3][2],
							m[1][0] * rhs.m[0][3] + m[1][1] * rhs.m[1][3] + m[1][2] * rhs.m[2][3] + m[1][3] * rhs.m[3][3],

							m[2][0] * rhs.m[0][0] + m[2][1] * rhs.m[1][0] + m[2][2] * rhs.m[2][0] + m[2][3] * rhs.m[3][0],
							m[2][0] * rhs.m[0][1] + m[2][1] * rhs.m[1][1] + m[2][2] * rhs.m[2][1] + m[2][3] * rhs.m[3][1],
							m[2][0] * rhs.m[0][2] + m[2][1] * rhs.m[1][2] + m[2][2] * rhs.m[2][2] + m[2][3] * rhs.m[3][2],
							m[2][0] * rhs.m[0][3] + m[2][1] * rhs.m[1][3] + m[2][2] * rhs.m[2][3] + m[2][3] * rhs.m[3][3],

							m[3][0] * rhs.m[0][0] + m[3][1] * rhs.m[1][0] + m[3][2] * rhs.m[2][0] + m[3][3] * rhs.m[3][0],
							m[3][0] * rhs.m[0][1] + m[3][1] * rhs.m[1][1] + m[3][2] * rhs.m[2][1] + m[3][3] * rhs.m[3][1],
							m[3][0] * rhs.m[0][2] + m[3][1] * rhs.m[1][2] + m[3][2] * rhs.m[2][2] + m[3][3] * rhs.m[3][2],
							m[3][0] * rhs.m[0][3] + m[3][1] * rhs.m[1][3] + m[3][2] * rhs.m[2][3] + m[3][3] * rhs.m[3][3]);
		}

		Matrix4 operator-() const
		{
			return Matrix4(-m[0][0], -m[0][1], -m[0][2], -m[0][3],
							-m[1][0], -m[1][1], -m[1][2], -m[1][3],
							-m[2][0], -m[2][1], -m[2][2], -m[2][3],
							-m[3][0], -m[3][1], -m[3][2], -m[3][3]);
		}

		Matrix4 Transpose() const
		{
			return Matrix4(m[0][0], m[1][0], m[2][0], m[3][0],
							m[0][1], m[1][1], m[2][1], m[3][1],
							m[0][2], m[1][2], m[2][2], m[3][2],
							m[0][3], m[1][3], m[2][3], m[3][3]);
		}

		// www.intel.com¿¡¼­ °¡Á®¿È
		Matrix4 Inverse() const
		{
			double a0 = m1[ 0] * m1[ 5] - m1[ 1] * m1[ 4];
			double a1 = m1[ 0] * m1[ 6] - m1[ 2] * m1[ 4];
			double a2 = m1[ 0] * m1[ 7] - m1[ 3] * m1[ 4];
			double a3 = m1[ 1] * m1[ 6] - m1[ 2] * m1[ 5];
			double a4 = m1[ 1] * m1[ 7] - m1[ 3] * m1[ 5];
			double a5 = m1[ 2] * m1[ 7] - m1[ 3] * m1[ 6];
			double b0 = m1[ 8] * m1[13] - m1[ 9] * m1[12];
			double b1 = m1[ 8] * m1[14] - m1[10] * m1[12];
			double b2 = m1[ 8] * m1[15] - m1[11] * m1[12];
			double b3 = m1[ 9] * m1[14] - m1[10] * m1[13];
			double b4 = m1[ 9] * m1[15] - m1[11] * m1[13];
			double b5 = m1[10] * m1[15] - m1[11] * m1[14];

			double det = a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;

			Matrix4 inverse;
			if(DBL_EPSILON > 0)
			{
				inverse.m1[ 0] = + m1[ 5]*b5 - m1[ 6]*b4 + m1[ 7]*b3;
				inverse.m1[ 4] = - m1[ 4]*b5 + m1[ 6]*b2 - m1[ 7]*b1;
				inverse.m1[ 8] = + m1[ 4]*b4 - m1[ 5]*b2 + m1[ 7]*b0;
				inverse.m1[12] = - m1[ 4]*b3 + m1[ 5]*b1 - m1[ 6]*b0;
				inverse.m1[ 1] = - m1[ 1]*b5 + m1[ 2]*b4 - m1[ 3]*b3;
				inverse.m1[ 5] = + m1[ 0]*b5 - m1[ 2]*b2 + m1[ 3]*b1;
				inverse.m1[ 9] = - m1[ 0]*b4 + m1[ 1]*b2 - m1[ 3]*b0;
				inverse.m1[13] = + m1[ 0]*b3 - m1[ 1]*b1 + m1[ 2]*b0;
				inverse.m1[ 2] = + m1[13]*a5 - m1[14]*a4 + m1[15]*a3;
				inverse.m1[ 6] = - m1[12]*a5 + m1[14]*a2 - m1[15]*a1;
				inverse.m1[10] = + m1[12]*a4 - m1[13]*a2 + m1[15]*a0;
				inverse.m1[14] = - m1[12]*a3 + m1[13]*a1 - m1[14]*a0;
				inverse.m1[ 3] = - m1[ 9]*a5 + m1[10]*a4 - m1[11]*a3;
				inverse.m1[ 7] = + m1[ 8]*a5 - m1[10]*a2 + m1[11]*a1;
				inverse.m1[11] = - m1[ 8]*a4 + m1[ 9]*a2 - m1[11]*a0;
				inverse.m1[15] = + m1[ 8]*a3 - m1[ 9]*a1 + m1[10]*a0;

				double invDet = 1.0/det;
				inverse.m1[ 0] *= invDet;
				inverse.m1[ 1] *= invDet;
				inverse.m1[ 2] *= invDet;
				inverse.m1[ 3] *= invDet;
				inverse.m1[ 4] *= invDet;
				inverse.m1[ 5] *= invDet;
				inverse.m1[ 6] *= invDet;
				inverse.m1[ 7] *= invDet;
				inverse.m1[ 8] *= invDet;
				inverse.m1[ 9] *= invDet;
				inverse.m1[10] *= invDet;
				inverse.m1[11] *= invDet;
				inverse.m1[12] *= invDet;
				inverse.m1[13] *= invDet;
				inverse.m1[14] *= invDet;
				inverse.m1[15] *= invDet;
			}

			return inverse;
		}

		Vector4 operator*(const Vector4 &rhs) const
		{
			return Vector4(this->m[0][0] * rhs.x + this->m[0][1] * rhs.y + this->m[0][2] * rhs.z + this->m[0][3] * rhs.w,
								this->m[1][0] * rhs.x + this->m[1][1] * rhs.y + this->m[1][2] * rhs.z + this->m[1][3] * rhs.w,
								this->m[2][0] * rhs.x + this->m[2][1] * rhs.y + this->m[2][2] * rhs.z + this->m[2][3] * rhs.w,
								this->m[3][0] * rhs.x + this->m[3][1] * rhs.y + this->m[3][2] * rhs.z + this->m[3][3] * rhs.w);
		}

		Matrix4& operator=(const Matrix4 &rhs)
		{
			this->m[0][0] = rhs.m[0][0]; this->m[0][1] = rhs.m[0][1]; this->m[0][2] = rhs.m[0][2]; this->m[0][3] = rhs.m[0][3];
			this->m[1][0] = rhs.m[1][0]; this->m[1][1] = rhs.m[1][1]; this->m[1][2] = rhs.m[1][2]; this->m[1][3] = rhs.m[1][3];
			this->m[2][0] = rhs.m[2][0]; this->m[2][1] = rhs.m[2][1];	this->m[2][2] = rhs.m[2][2]; this->m[2][3] = rhs.m[2][3];
			this->m[3][0] = rhs.m[3][0]; this->m[3][1] = rhs.m[3][1];	this->m[3][2] = rhs.m[3][2]; this->m[3][3] = rhs.m[3][3];
			return *this;
		}
	};

	/**
	 * @brief	Matrix3
	 * @author	Woonhyuk Baek
	 */
	struct Matrix3{
		union{
			struct{double m[3][3];};
			struct{double m1[9];};
			struct{double	_11, _12, _13, 
							_21, _22, _23,
							_31, _32, _33; 
					};
		};

		Matrix3()
		{
			m[0][0] = 0;	m[0][1] = 0;	m[0][2] = 0; 
			m[1][0] = 0;	m[1][1] = 0;	m[1][2] = 0;
			m[2][0] = 0;	m[2][1] = 0;	m[2][2] = 0;
		}
		
		
		Matrix3(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33)
		{
			m[0][0] = m11;	m[0][1] = m12;	m[0][2] = m13; 
			m[1][0] = m21;	m[1][1] = m22;	m[1][2] = m23;
			m[2][0] = m31;	m[2][1] = m32;	m[2][2] = m33;
		}

		Matrix3 operator+(const Matrix3 &rhs) const
		{
			return	Matrix3(m[0][0] + rhs.m[0][0],	m[0][1] + rhs.m[0][1],	m[0][2] + rhs.m[0][2], 
							m[1][0] + rhs.m[1][0],	m[1][1] + rhs.m[1][1],	m[1][2] + rhs.m[1][2],
							m[2][0] + rhs.m[2][0],	m[2][1] + rhs.m[2][1],	m[2][2] + rhs.m[2][2]);
		}

		Matrix3 operator-(const Matrix3 &rhs) const
		{
			return	Matrix3(m[0][0] - rhs.m[0][0],	m[0][1] - rhs.m[0][1],	m[0][2] - rhs.m[0][2], 
							m[1][0] - rhs.m[1][0],	m[1][1] - rhs.m[1][1],	m[1][2] - rhs.m[1][2],
							m[2][0] - rhs.m[2][0],	m[2][1] - rhs.m[2][1],	m[2][2] - rhs.m[2][2]);
		}

		Matrix3 operator*(const Matrix3 &rhs) const
		{
			return Matrix3(m[0][0] * rhs.m[0][0] + m[0][1] * rhs.m[1][0] + m[0][2] * rhs.m[2][0],
							m[0][0] * rhs.m[0][1] + m[0][1] * rhs.m[1][1] + m[0][2] * rhs.m[2][1],
							m[0][0] * rhs.m[0][2] + m[0][1] * rhs.m[1][2] + m[0][2] * rhs.m[2][2],
							m[1][0] * rhs.m[0][0] + m[1][1] * rhs.m[1][0] + m[1][2] * rhs.m[2][0],
							m[1][0] * rhs.m[0][1] + m[1][1] * rhs.m[1][1] + m[1][2] * rhs.m[2][1],
							m[1][0] * rhs.m[0][2] + m[1][1] * rhs.m[1][2] + m[1][2] * rhs.m[2][2],
							m[2][0] * rhs.m[0][0] + m[2][1] * rhs.m[1][0] + m[2][2] * rhs.m[2][0],
							m[2][0] * rhs.m[0][1] + m[2][1] * rhs.m[1][1] + m[2][2] * rhs.m[2][1],
							m[2][0] * rhs.m[0][2] + m[2][1] * rhs.m[1][2] + m[2][2] * rhs.m[2][2]);
		}

		Matrix3 Transpose() const
		{
			return Matrix3(m[0][0], m[1][0], m[2][0],
							m[0][1], m[1][1], m[2][1],
							m[0][2], m[1][2], m[2][2]);
		}

		Matrix3 Inverse() const
		{
			long double temp = (this->m[0][0] * (this->m[1][1] * this->m[2][2] - this->m[1][2] * this->m[2][1]) -
							this->m[0][1] * (this->m[1][0] * this->m[2][2] - this->m[1][2] * this->m[2][0]) +
							this->m[0][2] * (this->m[1][0] * this->m[2][1] - this->m[1][1] * this->m[2][1]));

			//if(temp == 0) return FALSE;

			long double detM = 1.0f / temp;
			return Matrix3((this->m[1][1] * this->m[2][2] - this->m[1][2] * this->m[2][1]) * detM,
							(this->m[0][2] * this->m[2][1] - this->m[0][1] * this->m[2][2]) * detM,
							(this->m[0][1] * this->m[1][2] - this->m[0][2] * this->m[1][1]) * detM,
							(this->m[1][2] * this->m[2][0] - this->m[1][0] * this->m[2][2]) * detM,
							(this->m[0][0] * this->m[2][2] - this->m[0][2] * this->m[2][0]) * detM,
							(this->m[0][2] * this->m[1][0] - this->m[0][0] * this->m[1][2]) * detM,
							(this->m[1][0] * this->m[2][1] - this->m[1][1] * this->m[2][0]) * detM,
							(this->m[0][1] * this->m[2][0] - this->m[0][0] * this->m[2][1]) * detM,
							(this->m[0][0] * this->m[1][1] - this->m[0][1] * this->m[1][0]) * detM);
		}

		Vector3 operator*(const Vector3 &rhs) const
		{
			return Vector3(this->m[0][0] * rhs.x + this->m[0][1] * rhs.y + this->m[0][2] * rhs.z,
							this->m[1][0] * rhs.x + this->m[1][1] * rhs.y + this->m[1][2] * rhs.z,
							this->m[2][0] * rhs.x + this->m[2][1] * rhs.y + this->m[2][2] * rhs.z);
		}

		Matrix3& operator=(const Matrix3 &rhs)
		{
			this->m[0][0] = rhs.m[0][0];	this->m[0][1] = rhs.m[0][1];	this->m[0][2] = rhs.m[0][2];
			this->m[1][0] = rhs.m[1][0];	this->m[1][1] = rhs.m[1][1];	this->m[1][2] = rhs.m[1][2];
			this->m[2][0] = rhs.m[2][0];	this->m[2][1] = rhs.m[2][1];	this->m[2][2] = rhs.m[2][2];
			return *this;
		}

	};

	/**
	 * @brief	Matrix2
	 * @author	Woonhyuk Baek
	 */
	struct Matrix2{
		union{
			struct{double m[2][2];};
			struct{double m1[4];};
			struct{double	_11, _12, 
							_21, _22;
					};
		};

		Matrix2()
		{
			m[0][0] = 0;	m[0][1] = 0;
			m[1][0] = 0;	m[1][1] = 0;
		}

		Matrix2(double m11, double m12, double m21, double m22)
		{
			m[0][0] = m11;	m[0][1] = m12;
			m[1][0] = m21;	m[1][1] = m22;
		}

		Matrix2 operator+(const Matrix2 &rhs) const
		{
			return Matrix2(this->m[0][0] + rhs.m[0][0],
							this->m[0][1] + rhs.m[0][1],
							this->m[1][0] + rhs.m[1][0],
							this->m[1][1] + rhs.m[1][1]);
		}

		Matrix2 operator-(const Matrix2 &rhs) const
		{
			return Matrix2(this->m[0][0] - rhs.m[0][0],
							this->m[0][1] - rhs.m[0][1],
							this->m[1][0] - rhs.m[1][0],
							this->m[1][1] - rhs.m[1][1]);
		}

		Matrix2 operator*(const Matrix2 &rhs) const
		{
			return Matrix2(this->m[0][0] * rhs.m[0][0] + this->m[0][1] * rhs.m[1][0],
							this->m[0][0] * rhs.m[0][1] + this->m[0][1] * rhs.m[1][1],
							this->m[1][0] * rhs.m[0][0] + this->m[1][1] * rhs.m[1][0],
							this->m[1][0] * rhs.m[0][1] + this->m[1][1] * rhs.m[1][1]);
		}

		Vector2 operator*(const Vector2 &rhs) const
		{
			return Vector2( this->m[0][0] * rhs.x + this->m[0][1] * rhs.y,
							this->m[1][0] * rhs.x + this->m[1][1] * rhs.y );
		}

		Matrix2 Transpose() const
		{
			return Matrix2(this->m[0][0], this->m[1][0], 
							this->m[0][1], this->m[1][1]);
		}

		Matrix2 Inverse() const
		{	
			double temp = this->m[0][0] * this->m[1][1] - this->m[0][1] * this->m[1][0];
			double detM = 1 / temp;
			return Matrix2(this->m[1][1] * detM, -(this->m[0][1] * detM), 
							-(this->m[1][0] * detM), this->m[0][0] * detM);
		}
	};
	/** @} */ // addtogroup Structures
}
#endif // _WBAEK_MATRIX_H_
