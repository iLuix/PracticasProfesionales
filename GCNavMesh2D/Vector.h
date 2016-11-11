/*
 * File:   Vector.h
 * Author: Luis
 *
 * Created on 19 de febrero de 2016, 05:26 PM
 */

#ifndef VECTOR_H
#define	VECTOR_H

#include <math.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <ostream>
//#include "LSE/PlatformDefines.h"

namespace LSE {
    namespace Util {
        namespace Math {
            template <int _N>
            class /*GC_LIBRARY_EXPORT*/ Vector {
            public:
                float data[_N];
                
                Vector();
                
                Vector(const float &x, const float &y);
                Vector(const float &x, const float &y, const float &z);
                Vector(const float &x, const float &y, const float &z, const float &w);
                
                explicit Vector(const float &f);
                
                float& operator[](int id) {
                    return data[id];
                }
                
                template <int _M>
                Vector<_M> toVector();
                
                constexpr float operator[](int id) const {
                    return data[id];
                }
                
                void operator=(Vector<_N> v) {
                    for (int i = 0; i < _N; i++){
                        data[i] = v[i];
                    }
                }
                bool operator==(const Vector<_N> &v);
                bool operator!=(const Vector<_N> &v);
                Vector<_N> operator+(const float &f);
                Vector<_N> operator-(const float &f);
                Vector<_N> operator*(const float &f);
                Vector<_N> operator/(const float &f);
                Vector<_N> operator+(const Vector<_N> &v);
                Vector<_N> operator-(const Vector<_N> &v);
                Vector<_N> operator*(const Vector<_N> &v);
                Vector<_N> operator/(const Vector<_N> &v);
                Vector<_N>(Vector<_N - 1 > orig);
                Vector<_N>(Vector<_N + 1 > orig);
                void operator+=(const float &f);
                void operator-=(const float &f);
                void operator*=(const float &f);
                void operator/=(const float &f);
                void operator+=(const Vector<_N> &v);
                void operator-=(const Vector<_N> &v);
                void operator*=(const Vector<_N> &v);
                void operator/=(const Vector<_N> &v);
                float dot(const Vector<_N> &v);
                float length2() const;
                float length() const;
                Vector<_N> getNormalized();
                Vector<_N> clamp(float m, float M);
                ///cross product
                Vector < 3 > operator ^(const Vector < 3 > &v);
                //// Para los colores
                static Vector<3> fromHexColor(std::string hex);
                Vector <1> x();
                Vector <2> xy();
                Vector <3> xyz();
                Vector<_N> clampLengthUpTo(double maxLen);
#ifndef __ORBIS__
                template<int N>
                friend std::wostream& operator<<(std::wostream&, Vector<N>&);
#else
                template<int N>
                friend std::ostream& operator<<(std::ostream&, Vector<N>);
#endif
            };
#ifndef __ORBIS__
            template<int N> std::wostream& operator<<(std::wostream& out, Vector<N>& vec){
                for(int i=0;i<N;i++){
                    out << vec.data[i] << " ";
                }
                return out;
            }
#else
            template<int N> std::ostream& operator<<(std::ostream& out, Vector<N> vec){
                for(int i=0;i<N;i++){
                    out << vec.data[i] << " ";
                }
                return out;
            }
#endif
            template<int N> Vector<N> operator*(double a, Vector<N> v) {
                return v*a;
            }
            template<int N> Vector<N> operator-(Vector<N> v){
                return v*(-1.0);
            }
            
            template <>
            class /*GC_LIBRARY_EXPORT*/ Vector<0> {
            public:
                float data[1];
            };
            
            template<>
            inline Vector<2>::Vector(const float& x, const float& y)
            {
                data[0] = x;
                data[1] = y;
            }
            template<>
            inline Vector<3>::Vector(const float& x, const float& y, const float& z)
            {
                data[0] = x;
                data[1] = y;
                data[2] = z;
            }
            template<>
            inline Vector<4>::Vector(const float& x, const float& y, const float& z, const float& w)
            {
                data[0] = x;
                data[1] = y;
                data[2] = z;
                data[3] = w;
            }
            
        }
    }
}

#endif	/* VECTOR_H */