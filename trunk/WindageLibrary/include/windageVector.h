//----------------------------------------------------------------
// 2,3,4D 벡터 클래스
//----------------------------------------------------------------

#ifndef __VECTOR__
#define __VECTOR__

#include <math.h>

// 4D벡터클래스
struct Vector4{
	union{
		struct{double x, y, z, w;};
		struct{double v[4];};
	};

	// 생성자
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

	// + 연산
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

	// - 연산
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

	// 내적(Dot)
	double operator*(const Vector4 &rhs) const
	{
		return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
	}

	// 외적(Cross)
	Vector4 operator^(const Vector4 &rhs) const
	{
		return Vector4(this->y*rhs.z - this->z*rhs.y,
						 this->z*rhs.x - this->x*rhs.z,
						 this->x*rhs.y - this->y*rhs.x, 1);
	}

	// 복사
	void operator=(const Vector4 rhs)
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		this->w = rhs.w;
	}

	// -기호
	Vector4 operator-() const
	{
		return Vector4(-this->x, -this->y, -this->z, -this->w);
	}
	
	// 스칼라와 곱
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

	// 스칼라와 나누기
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

	// 비교
	bool operator==(const Vector4 &rhs) const
	{
		if(this->x == rhs.x && this->y == rhs.y && this->z == rhs.z)
			return true;
		else
			return false;
	}

	// 길이값 계산
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

// 3D벡터클래스
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

	// 길이값 계산
	double getLength() const
	{
		return sqrt( pow(this->x , 2) + pow(this->y, 2) + pow(this->z, 2) ); 
	}

	// distance
	double getDistance(const Vector3 &rhs) const
	{
		return sqrt( pow(this->x - rhs.x, 2) + pow(this->y - rhs.y, 2) + pow(this->z - rhs.z, 2));
	}

	// set scale value to z
	double setScaleValue()
	{
		this->z = sqrt( pow(this->x , 2) + pow(this->y, 2) ); 
	}

	// initialize
	void initialize()
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
};

// 2D벡터클래스
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

	// 길이값 계산
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
		this->y = 0;
	}
};

#endif