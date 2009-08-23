/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo
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

//----------------------------------------------------------------
// 2x2,3x3,4x4 행렬 클래스
//----------------------------------------------------------------
#include "memory.h"
#include "wVector.h"

#ifndef _WBAEK_MATRIX_H_
#define _WBAEK_MATRIX_H_

namespace windage
{
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

		// www.intel.com에서 가져옴
		Matrix4 Inverse() const
		{
			double tmp[12];
			double src[16];
			double dst[16];
			double det;

			memcpy(&src, this, 4*16);

			tmp[ 0] = src[10] * src[15];
			tmp[ 1] = src[11] * src[14];
			tmp[ 2] = src[ 9] * src[15];
			tmp[ 3] = src[11] * src[13];
			tmp[ 4] = src[ 9] * src[14];
			tmp[ 5] = src[10] * src[13];
			tmp[ 6] = src[ 8] * src[15];
			tmp[ 7] = src[11] * src[12];
			tmp[ 8] = src[ 8] * src[14];
			tmp[ 9] = src[10] * src[12];
			tmp[10] = src[ 8] * src[13];
			tmp[11] = src[ 9] * src[12];

			dst[0]  = tmp[0] * src[5] + tmp[3] * src[6] + tmp[ 4] * src[7];
			dst[0] -= tmp[1] * src[5] + tmp[2] * src[6] + tmp[ 5] * src[7];
			dst[1]  = tmp[1] * src[4] + tmp[6] * src[6] + tmp[ 9] * src[7];
			dst[1] -= tmp[0] * src[4] + tmp[7] * src[6] + tmp[ 8] * src[7];
			dst[2]  = tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7];
			dst[2] -= tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7];
			dst[3]  = tmp[5] * src[4] + tmp[8] * src[5] + tmp[11] * src[6];
			dst[3] -= tmp[4] * src[4] + tmp[9] * src[5] + tmp[10] * src[6];
			dst[4]  = tmp[1] * src[1] + tmp[2] * src[2] + tmp[ 5] * src[3];
			dst[4] -= tmp[0] * src[1] + tmp[3] * src[2] + tmp[ 4] * src[3];
			dst[5]  = tmp[0] * src[0] + tmp[7] * src[2] + tmp[ 8] * src[3];
			dst[5] -= tmp[1] * src[0] + tmp[6] * src[2] + tmp[ 9] * src[3];
			dst[6]  = tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3];
			dst[6] -= tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3];
			dst[7]  = tmp[4] * src[0] + tmp[9] * src[1] + tmp[10] * src[2];
			dst[7] -= tmp[5] * src[0] + tmp[8] * src[1] + tmp[11] * src[2];

			tmp[ 0] = src[ 2] * src[ 7];
			tmp[ 1] = src[ 3] * src[ 6];
			tmp[ 2] = src[ 1] * src[ 7];
			tmp[ 3] = src[ 3] * src[ 5];
			tmp[ 4] = src[ 1] * src[ 6];
			tmp[ 5] = src[ 2] * src[ 5];
			tmp[ 6] = src[ 0] * src[ 7];
			tmp[ 7] = src[ 3] * src[ 4];
			tmp[ 8] = src[ 0] * src[ 6];
			tmp[ 9] = src[ 2] * src[ 4];
			tmp[10] = src[ 0] * src[ 5];
			tmp[11] = src[ 1] * src[ 4];

			dst[ 8]  = tmp[ 0] * src[13] + tmp[ 3] * src[14] + tmp[ 4] * src[15];
			dst[ 8] -= tmp[ 1] * src[13] + tmp[ 2] * src[14] + tmp[ 5] * src[15];
			dst[ 9]  = tmp[ 1] * src[12] + tmp[ 6] * src[14] + tmp[ 9] * src[15];
			dst[ 9] -= tmp[ 0] * src[12] + tmp[ 7] * src[14] + tmp[ 8] * src[15];
			dst[10]  = tmp[ 2] * src[12] + tmp[ 7] * src[13] + tmp[10] * src[15];
			dst[10] -= tmp[ 3] * src[12] + tmp[ 6] * src[13] + tmp[11] * src[15];
			dst[11]  = tmp[ 5] * src[12] + tmp[ 8] * src[13] + tmp[11] * src[14];
			dst[11] -= tmp[ 4] * src[12] + tmp[ 9] * src[13] + tmp[10] * src[14];
			dst[12]  = tmp[ 2] * src[10] + tmp[ 5] * src[11] + tmp[ 1] * src[ 9];
			dst[12] -= tmp[ 4] * src[11] + tmp[ 0] * src[ 9] + tmp[ 3] * src[10];
			dst[13]  = tmp[ 8] * src[11] + tmp[ 0] * src[ 8] + tmp[ 7] * src[10];
			dst[13] -= tmp[ 6] * src[10] + tmp[ 9] * src[11] + tmp[ 1] * src[ 8];
			dst[14]  = tmp[ 6] * src[ 9] + tmp[11] * src[11] + tmp[ 3] * src[ 8];
			dst[14] -= tmp[10] * src[11] + tmp[ 2] * src[ 8] + tmp[ 7] * src[ 9];
			dst[15]  = tmp[10] * src[10] + tmp[ 4] * src[ 8] + tmp[ 9] * src[ 9];
			dst[15] -= tmp[ 8] * src[ 9] + tmp[11] * src[10] + tmp[ 5] * src[ 8];

			det = src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3];
			if(!(det == 0.0f))
			{
				det = 1/det;
			}
			else
			{
				det = 1.0f;
			}

			for(int j=0; j<16; ++j)
			{
				dst[j] *= det;
			}

			Matrix4 Result;

			memcpy(&Result, &dst, 16*4);

			return Result.Transpose();
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
}
#endif
