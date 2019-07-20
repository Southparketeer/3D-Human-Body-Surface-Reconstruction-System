#pragma once
#include <math.h>



namespace LUY_VEC{
	// Vectors 
	template <typename T> 
	class vec2
	{
	public:
		T x;
		T y;

		vec2()
		{
			x = y = 0;
		}
		vec2(T vecX, T vecY)
		{
			x = vecX;
			y = vecY;
		}
		bool vec2<T>::operator==(vec2<T> pt)
		{
			if( x == pt.x && y == pt.y) return true;
			else return false;
		}
	};

	template <typename T> 
	class vec3
	{
	public:
		T x, y, z;

		vec3<T>()
		{
			x = y = z = 0;
		}

		vec3<T>(T e0, T e1, T e2) 
		{
			x = e0; y = e1; z = e2;
		}

		vec3<T>(const vec3<T> & e)
		{
			x = e.x;
			y = e.y;
			z = e.z;
		}
		void print()
		{
			printf("%.3f\t%.3f\t%.3f\t\n", this->x, this->y, this->z);
		}
		void Set(T e0, T e1, T e2) 
		{
			x = e0; y = e1; z = e2;
		}

		vec3<T> operator+(const T &f) 
		{
			return vec3<T>( this->x + f, this->y + f, this->z + f );
		}

		vec3<T> operator-(const T &f) 
		{
			return vec3<T>( this->x - f, this->y - f, this->z - f );
		}

		vec3<T> operator*(const T &f) 
		{
			return vec3<T>( this->x * f, this->y * f, this->z * f );
		}

		vec3<T> operator/(const T &f) 
		{
			return vec3<T>( this->x / f, this->y / f, this->z / f);
		}

		vec3<T>& operator+=(const vec3<T>& lhs)
		{
			Set(x + lhs.x, y + lhs.y, z + lhs.z);
			return *this;
		}

		vec3<T>& operator-=(const vec3<T>& lhs)
		{
			Set(x - lhs.x, y - lhs.y, z - lhs.z);
			return *this;
		}

		vec3<T>& operator*=(const vec3<T>& lhs)
		{
			Set(x * lhs.x, y * lhs.y, z * lhs.z);
			return *this;
		}

		vec3<T>& operator/=(const vec3<T>& lhs)
		{
			Set(x / lhs.x, y / lhs.y, z / lhs.z);
			return *this;
		}

		bool equal(const vec3<T>& v)
		{
			if(this->x == v.x && this->y == v.y && this->z == v.z)
				return true;
			return false;
		}

		float length() const
		{
			return sqrtf(x*x +y*y+z*z);
		}

		void normalize() 
		{
			Set(this->x/this->length(), this->y/this->length(),	this->z/this->length());
		}

		T dot( const vec3<T> &v)
		{
			return v.x * this->x + v.y * this->y + v.z * this->z;
		}

		vec3<T> cross(const vec3<T> &v)
		{
			return vec3<T> ((this->y * v.z - this->z * v.y),	(this->z * v.x - this->x * v.z),	(this->x * v.y - this->y * v.x));
		}

		T Max()
		{
			T max = 0;
			this->x > this->y ? max = this->x : max = this->y;
			max > this->z ? max = max : max = this->z;
			return max;
		}

		T Max( int& idx )
		{
			T max = 0;
			this->x > this->y ? max = this->x : max = this->y;
			this->x > this->y ? idx = 1 : idx = 2;
			max > this->z ? max = max : max = this->z;
			max == this->z ? idx = 3 : idx = idx;
			return max;
		}

		T Min()
		{
			T min;
			this->x < this->y ? min = this->x : min = this->y;
			min < this->z ? min = min : min = this->z;
			return min;
		}

		T Min( int& idx)
		{
			T min;
			this->x < this->y ? min = this->x : min = this->y;
			this->x < this->y ? idx = 1 : idx = 2;
			min < this->z ? min = min : min = this->z;
			min == this->z ? idx = 3 : idx = idx;
			return min;
		}

		inline void Rotate( _Out_cap_c_(9) const T* rot) // rot 3*3
		{
			vec3f t;
			t.x = rot[0] * this->x + rot[1] * this->y + rot[2] * this->z;
			t.y = rot[3] * this->x + rot[4] * this->y + rot[5] * this->z;
			t.z = rot[6] * this->x + rot[7] * this->y + rot[8] * this->z;
			(*this) = t;
		}

	};

	template <typename T> 
	T inner(const vec3<T> &v1, const vec3<T> &v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
	template <typename T> 
	vec3<T> operator+(const vec3<T> &v1, const vec3<T> &v2) 
	{
		return vec3<T>( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
	}
	template <typename T> 
	vec3<T> operator-(const vec3<T> &v1, const vec3<T> &v2) 
	{
		return vec3<T>( v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	}
	template <typename T> 
	vec3<T> operator*(const vec3<T> &v1, const vec3<T> &v2) 
	{
		return vec3<T>( v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
	}
	template <typename T> 
	vec3<T> operator/(const vec3<T> &v1, const vec3<T> &v2) 
	{
		return vec3<T>( v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
	}

	//template <typename T> 
	//vec3<T> cross(const vec3<T> &v1, const vec3<T> &v2) 
	//{
	//	return vec3<T>(	(v1.y*v2.z - v1.z*v2.y),
	//		(v1.z*v2.x - v1.x*v2.z),
	//		(v1.x*v2.y - v1.y*v2.x) );
	//}



	typedef vec3<float> vec3f;
	typedef vec3<int> vec3i;
	typedef vec2<float> vec2f;
	typedef vec2<int> vec2i;



	struct Vertex{
		vec3f pos;
		vec3f nor;
		vec3f col;
		Vertex()
		{
			pos.Set(0,0,0);
			nor.Set(0,0,0);
			col.Set(0,0,0);
		}
		Vertex(const Vertex& v)
		{
			this->pos = v.pos;
			this->nor = v.nor;
			this->col = v.col;
		}
	};

	struct PAxis{
		vec3f Axis_X;
		vec3f Axis_Y;
		vec3f Axis_Z;
		vec3f Eigen;
		PAxis()
		{
			Axis_X.Set(1,0,0);
			Axis_Y.Set(0,1,0);
			Axis_Z.Set(0,0,1);
			Eigen.Set(1,1,1);
		}
		PAxis(const PAxis & in)
		{
			this->Axis_X = in.Axis_X;
			this->Axis_Y = in.Axis_Y;
			this->Axis_Z = in.Axis_Z;
			this->Eigen  = in.Eigen;
		}				   
	};
}