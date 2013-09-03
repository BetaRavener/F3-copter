/*
* F3-copter - STM32-F3 Discovery based tricopter
* Copyright (c) 2013 Ivan Sevcik - ivan-sevcik@hotmail.com
*
* This software is provided 'as-is', without any express or
* implied warranty. In no event will the authors be held
* liable for any damages arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute
* it freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented;
*    you must not claim that you wrote the original software.
*    If you use this software in a product, an acknowledgment
*    in the product documentation would be appreciated but
*    is not required.
*
* 2. Altered source versions must be plainly marked as such,
*    and must not be misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any
*    source distribution.
*/

#include <cmath>
#include <cstdlib>

namespace math3d{
    
    template <typename T>
    Vector3<T>::Vector3() /*:
    x(_vec[0]),
    y(_vec[1]),
    z(_vec[2])*/
    {
        _vec[0] = 0;
        _vec[1] = 0;
        _vec[2] = 0;
    }
    
    template <typename T>
    Vector3<T>::Vector3(T nx, T ny, T nz) /*: 
    x(_vec[0]),
    y(_vec[1]),
    z(_vec[2])*/
    {
        _vec[0] = nx;
        _vec[1] = ny;
        _vec[2] = nz;
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>::Vector3(const Vector3<U>& b) /*:
    x(_vec[0]),
    y(_vec[1]),
    z(_vec[2])*/
    {
        _vec[0] = b[0];
        _vec[1] = b[1];
        _vec[2] = b[2];
    }
    
    template <typename T>
    T& Vector3<T>::operator [](unsigned int i)
    {
        if (i < 3)
            return _vec[i];
        T* null = 0;
        return *null;
    }
    
    template <typename T>
    const T& Vector3<T>::operator [](unsigned int i) const
    {
        if (i < 3)
            return _vec[i];
        T* null = 0;
        return *null;
    }
    
    template <typename T>
    T& Vector3<T>::operator ()(unsigned int i)
    {
        if (i < 3)
            return _vec[i];
        T* null = 0;
        return *null;
    }
    
    template <typename T>
    const T& Vector3<T>::operator ()(unsigned int i) const
    {
        if (i < 3)
            return _vec[i];
        T* null = 0;
        return *null;
    }
    
    template <typename T>
    Vector3<T>& Vector3<T>::operator = (const Vector3<T>& b)
    {
        _vec[0] = b[0];
        _vec[1] = b[1];
        _vec[2] = b[2];
        return *this;
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>& Vector3<T>::operator = (const Vector3<U>& b)
    {
        _vec[0] = b[0];
        _vec[1] = b[1];
        _vec[2] = b[2];
        return *this;
    }
    
    template <typename T>
    Vector3<T> Vector3<T>::operator -() const
    {
        return Vector3<T>(-_vec[0], -_vec[1], -_vec[2]);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator + (const Vector3<U>& b) const
    {
        return Vector3<T>(_vec[0] + b[0],
                          _vec[1] + b[1],
                          _vec[2] + b[2]);
    }
    
    
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator - (const Vector3<U>& b) const
    {
        return Vector3<T>(_vec[0] - b[0],
                          _vec[1] - b[1],
                          _vec[2] - b[2]);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator * (const Vector3<U>& b) const
    {
        return Vector3<T>(_vec[0] * b[0],
                          _vec[1] * b[1],
                          _vec[2] * b[2]);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator / (const Vector3<U>& b) const
    {
        return Vector3<T>(_vec[0] / b[0],
                          _vec[1] / b[1],
                          _vec[2] / b[2]);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>& Vector3<T>::operator += (const Vector3<U>& b)
    {
        _vec[0] += b[0]; 
        _vec[1] += b[1]; 
        _vec[2] += b[2];
        return *this;
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>& Vector3<T>::operator -= (const Vector3<U>& b)
    {
        _vec[0] -= b[0];
        _vec[1] -= b[1];
        _vec[2] -= b[2];
        return *this;
    }
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator * (const U b) const
    {
        return Vector3<T>(_vec[0] * b,
                          _vec[1] * b,
                          _vec[2] * b);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T> Vector3<T>::operator / (const U b) const
    {
        return Vector3<T>(_vec[0] / b,
                          _vec[1] / b, 
                          _vec[2] / b);
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>& Vector3<T>::operator *= (const U b)
    {
        _vec[0] *= b;
        _vec[1] *= b;
        _vec[2] *= b;
        return *this;
    }
    
    template <typename T>
    template <typename U>
    Vector3<T>& Vector3<T>::operator /= (const U b)
    {
        _vec[0] /= b; 
        _vec[1] /= b; 
        _vec[2] /= b;
        return *this;
    }

    template <typename T>
    bool Vector3<T>::operator == (const Vector3<T>& b) const
    {
        return _vec[0] == b[0] && _vec[1] == b[1] && _vec[2] == b[2];
    }
    
    template <typename T>
    bool Vector3<T>::operator == (const T b) const
    {
        return _vec[0] == b && _vec[1] == b && _vec[2] == b;
    }
    
    template <typename T>
    bool Vector3<T>::operator != (const Vector3<T>& b) const
    {
        return _vec[0] != b[0] || _vec[1] != b[1] || _vec[2] != b[2];
    }
    
    template <typename T>
    Vector3<T>& Vector3<T>::zero()
    {
        _vec[0] = _vec[1] = _vec[2] = 0;
        return *this;
    }

	template <typename T>
	Vector3<T> Vector3<T>::reflect (const Vector3<T>& n) const
    {
		return n * -2.0f * dotProduct(n) + *this;
	}

	template <typename T>
	Vector3<T> Vector3<T>::crossProduct (const Vector3<T>& b) const
    {
		return Vector3<T>(_vec[1] * b[2] - _vec[2] * b[1],
                          _vec[2] * b[0] - _vec[0] * b[2],
                          _vec[0] * b[1] - _vec[1] * b[0]);
	}

	template <typename T>
	T Vector3<T>::dotProduct (const Vector3<T>& b) const
    {
		return _vec[0] * b[0] + _vec[1] * b[1] + _vec[2] * b[2];
	}

	template <typename T>
	Vector3<T> Vector3<T>::normalize()
    {
		return *this / magnitude();
	}

	template <typename T>
	T Vector3<T>::magnitude() const
    {
		return sqrt(_vec[0] * _vec[0] + _vec[1] * _vec[1] + _vec[2] * _vec[2]);
	}

	template <typename T>
	Vector3<T> Vector3<T>::rotateX(const float radians) const
	{
        return Vector3<T>(_vec[0],
                          _vec[1] * std::cos(radians)  + _vec[2] * std::sin(radians),
                          _vec[1] * -std::sin(radians) + _vec[2] * std::cos(radians));
	}

	template <typename T>
	Vector3<T> Vector3<T>::rotateY(const float radians) const
	{
        return Vector3<T>(_vec[0] * std::cos(radians) + _vec[2] * -std::sin(radians),
                          _vec[1],
                          _vec[0] * std::sin(radians) + _vec[2] * std::cos(radians));
	}
}
