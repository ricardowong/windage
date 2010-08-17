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
 * @file	Vector.h
 * @author	Woonhyuk Baek
 * @brief	2x1, 3x1, 4x1 Vector structures
 */

#ifndef _WBAEK_VECTOR_H_
#define _WBAEK_VECTOR_H_

#include <float.h>
#include <math.h>
#define max(a,b)            (((a) > (b)) ? (a) : (b))

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
	 * @brief	Vector4
	 * @author	Woonhyuk Baek
	 */
	struct Vector4{
		union{
			struct{double x, y, z, w;};
			struct{double v[4];};
		};

		Vector4():x(0.0f), y(0.0f), z(0.0f), w(1.0f){}

		Vector4(double x, double y, double z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = 1;
		}

		Vector4(double x, double y, double z, double w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		Vector4 operator+(const Vector4 &rhs) const
		{
			return Vector4(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z, this->w + rhs.w);
		}

		void operator+=(const Vector4 &rhs)
		{
			this->x += rhs.x;
			this->y += rhs.y;
			this->z += rhs.z;
			this->w += rhs.w;
		}

		Vector4 operator-(const Vector4 &rhs) const
		{
			return Vector4(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z, this->w - rhs.w);
		}

		void operator-=(const Vector4 &rhs)
		{
			this->x -= rhs.x;
			this->y -= rhs.y;
			this->z -= rhs.z;
			this->w -= rhs.w;
		}

		// Dot
		double operator*(const Vector4 &rhs) const
		{
			return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
		}

		// Cross
		Vector4 operator^(const Vector4 &rhs) const
		{
			return Vector4(this->y*rhs.z - this->z*rhs.y,
							 this->z*rhs.x - this->x*rhs.z,
							 this->x*rhs.y - this->y*rhs.x, 1);
		}

		void operator=(const Vector4 rhs)
		{
			this->x = rhs.x;
			this->y = rhs.y;
			this->z = rhs.z;
			this->w = rhs.w;
		}

		Vector4 operator-() const
		{
			return Vector4(-this->x, -this->y, -this->z, -this->w);
		}
		
		Vector4 operator*(const double s) const
		{
			return Vector4(this->x * s, this->y * s, this->z * s, this->w * s);
		}

		void operator*=(const double s)
		{
			this->x *= s;
			this->y *= s;
			this->z *= s;
			this->w *= s;
		}

		Vector4 operator/(const double s) const
		{
			return Vector4(this->x / s, this->y / s, this->z / s, this->w / s);
		}

		void operator/=(const double s)
		{
			this->x /= s;
			this->y /= s;
			this->z /= s;
			this->w /= s;
		}

		bool operator==(const Vector4 &rhs) const
		{
			if(this->x == rhs.x && this->y == rhs.y && this->z == rhs.z)
				return true;
			else
				return false;
		}

		double getLength() const
		{
			return sqrt( pow(this->x , 2) + pow(this->y, 2) + pow(this->z, 2) + pow(this->w, 2) ); 
		}

		// distance
		double getDistance(const Vector4 &rhs) const
		{
			return sqrt( pow(this->x - rhs.x, 2) + pow(this->y - rhs.y, 2) + pow(this->z - rhs.z, 2) + pow(this->w - rhs.w, 2) );
		}

		// set scale value to w
		double setScaleValue()
		{
			this->w = sqrt( pow(this->x , 2) + pow(this->y, 2) + pow(this->z, 2) ); 
		}

		// initialize
		void initialize()
		{
			this->x = 0;
			this->y = 0;
			this->z = 0;
			this->w = 0;
		}
	};

	/**
	 * @brief	Vector3
	 * @author	Woonhyuk Baek
	 */
	struct Vector3{
		union{
			struct{double x, y, z;};
			struct{double v[3];};
		};

		Vector3():x(0.0f), y(0.0f), z(0.0f){}

		Vector3(double x, double y)
		{
			this->x = x;
			this->y = y;
			this->z = 0;
		}

		Vector3(double x, double y, double z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		Vector3 operator+(const Vector3 rhs) const
		{
			return Vector3(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z);
		}

		void operator+=(const Vector3 &rhs)
		{
			this->x += rhs.x;
			this->y += rhs.y;
			this->z += rhs.z;
		}

		Vector3 operator-(const Vector3 rhs) const
		{
			return Vector3(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z);
		}

		void operator-=(const Vector3 &rhs)
		{
			this->x -= rhs.x;
			this->y -= rhs.y;
			this->z -= rhs.z;
		}

		double operator*(const Vector3 rhs) const
		{
			return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
		}

		Vector3 operator^(const Vector3 rhs) const
		{
			return Vector3(this->y*rhs.z - this->z*rhs.y,
							 this->z*rhs.x - this->x*rhs.z,
							 this->x*rhs.y - this->y*rhs.x);
		}

		void operator=(const Vector3 rhs)
		{
			this->x = rhs.x;
			this->y = rhs.y;
			this->z = rhs.z;
			//return *this;
		}

		Vector3 operator-() const
		{
			return Vector3(-this->x, -this->y, -this->z);
		}
		
		Vector3 operator*(const double s) const
		{
			return Vector3(this->x * s, this->y * s, this->z * s);
		}

		void operator*=(const double s)
		{
			this->x *= s;
			this->y *= s;
			this->z *= s;
		}

		Vector3 operator/(const double s) const
		{
			return Vector3(this->x / s, this->y / s, this->z / s);
		}

		void operator/=(const double s)
		{
			this->x /= s;
			this->y /= s;
			this->z /= s;
		}

		double getLength() const
		{
			return sqrt( pow(this->x , 2) + pow(this->y, 2) + pow(this->z, 2) ); 
		}

		// distance
		double getDistance(const Vector3 &rhs) const
		{
			return sqrt( pow(this->x - rhs.x, 2) + pow(this->y - rhs.y, 2) + pow(this->z - rhs.z, 2) );
		}

		// set scale value to z
		double setScaleValue()
		{
			this->z = sqrt( pow(this->x, 2) + pow(this->y, 2) ); 
		}

		// initialize
		void initialize()
		{
			this->x = 0;
			this->y = 0;
			this->z = 0;
		}


		double getMax()
		{
			return max(max(this->x, this->y), this->z);
		}
	};

	/**
	 * @brief	Vector2
	 * @author	Woonhyuk Baek
	 */
	struct Vector2{
		union{
			struct{double x, y;};
			struct{double v[2];};
		};

		Vector2():x(0.0f), y(0.0f){}

		Vector2(double x, double y)
		{
			this->x = x;
			this->y = y;
		}

		Vector2 operator+(const Vector2 rhs) const
		{
			return Vector2(this->x + rhs.x, this->y + rhs.y);
		}

		void operator+=(const Vector2 &rhs)
		{
			this->x += rhs.x;
			this->y += rhs.y;
		}

		Vector2 operator-(const Vector2 rhs) const
		{
			return Vector2(this->x - rhs.x, this->y - rhs.y);
		}

		void operator-=(const Vector2 &rhs)
		{
			this->x -= rhs.x;
			this->y -= rhs.y;
		}

		double operator*(const Vector2 rhs) const
		{
			return this->x * rhs.x + this->y * rhs.y;
		}

		Vector2 operator/(const Vector2 rhs) const
		{
			return Vector2(this->x / rhs.x, this->y / rhs.y);
		}

		Vector2 operator*(const double rhs) const
		{
			return Vector2(this->x * rhs, this->y * rhs);
		}

		void operator*=(const double s)
		{
			this->x *= s;
			this->y *= s;
		}

		Vector2 operator/(const double rhs) const
		{
			return Vector2(this->x / rhs, this->y / rhs);
		}

		void operator/=(const double s)
		{
			this->x /= s;
			this->y /= s;
		}

		// cross
		Vector3 operator^(const Vector2 rhs) const
		{
			return Vector3(0, 0, this->x * rhs.y - this->y * rhs.x);
		}

		void operator=(const Vector2 rhs)
		{
			this->x = rhs.x;
			this->y = rhs.y;

			//return *this;
		}

		Vector2 operator-() const
		{
			return Vector2(-this->x, -this->y);
		}

		double getLength() const
		{
			return sqrt( pow(this->x , 2) + pow(this->y, 2)); 
		}

		// distance
		double getDistance(const Vector2 &rhs) const
		{
			return sqrt( pow(this->x - rhs.x, 2) + pow(this->y - rhs.y, 2) );
		}

		// initialize
		void initialize()
		{
			this->x = 0;
		}
	};
	/** @} */ // addtogroup Structures
}
#endif // _WBAEK_VECTOR_H_