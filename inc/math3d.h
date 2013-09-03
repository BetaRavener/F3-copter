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

#pragma once
#ifndef MATH_3D
#define MATH_3D

namespace math3d{

    static const double Pi = 3.1415926535897932384626433832795029;
    static const double Pi_2 = 1.5707963267948966192313216916397514;
    
    static const double degreesInRadian = 180.0 / Pi;
    static const double radiansInDegree = Pi / 180.0;

    template <typename T>
    static T Radians(T degrees){
        return degrees * radiansInDegree;
    }
    
    template <typename T>
    static T Degrees(T radians){
        return radians * degreesInRadian;
    }

    template <typename T>
    class Vector3
    {
    public:
        /*T& x;
        T& y;
        T& z;*/

        Vector3();
            
        Vector3(T nx, T ny, T nz);
            
        template <typename U>
        Vector3(const Vector3<U>& b);
            
        T& operator [](unsigned int i);
        const T& operator [](unsigned int i) const;

        T& operator ()(unsigned int i);
        const T& operator ()(unsigned int i) const;
        
        Vector3<T>& operator =(const Vector3<T>& b);

        template <typename U>
        Vector3<T>& operator =(const Vector3<U>& b);
           
        Vector3<T> operator -() const;
        
        template <typename U>
        Vector3<T> operator +(const Vector3<U>& b) const;
        
        template <typename U>
		Vector3<T> operator -(const Vector3<U>& b) const;

        template <typename U>
		Vector3<T> operator *(const Vector3<U>& b) const;

		template <typename U>
		Vector3<T> operator /(const Vector3<U>& b) const;

        template <typename U>
        Vector3<T>& operator +=(const Vector3<U>& b);

        template <typename U>
        Vector3<T>& operator -=(const Vector3<U>& b);

        template <typename U>
        Vector3<T> operator *(const U b) const;
        
        template <typename U>
        Vector3<T> operator /(const U b) const;
        
        template <typename U>
        Vector3<T>& operator *=(const U b);
        
        template <typename U>
        Vector3<T>& operator /=(const U b);

        bool operator ==(const Vector3<T>& b) const;
        bool operator ==(const T b) const;
        bool operator !=(const Vector3<T>& b) const;

        Vector3<T>&  zero();
        
        Vector3<T> normalize();
        T magnitude() const;
        T dotProduct (const Vector3<T>& b) const;
        Vector3<T> crossProduct (const Vector3<T>& b) const;
        Vector3<T> reflect (const Vector3<T>& n) const;

        Vector3<T> rotateX(const float radians) const;
        Vector3<T> rotateY(const float radians) const;
    private:
        T _vec[3];
    };

    static const Vector3<int> ZeroVector(0,0,0);
    static const Vector3<int> X_Axis(1,0,0);
    static const Vector3<int> Y_Axis(0,1,0);
    static const Vector3<int> Z_Axis(0,0,1);

    
    template <class T>
    inline T getSign(T a)
    {
        return a > 0 ? 1 : a < 0 ? -1 : 0;
    }
    
    template <class T>
    inline T mod(const float &a, const float &b)
    {
        return a - ((int)(a / b)) * b;
    }

}

#include "math3d.inl"

#endif
