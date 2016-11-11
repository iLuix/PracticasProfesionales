/*
 * File:   Vector.cpp
 * Author: Luis
 *
 * Created on 19 de febrero de 2016, 05:26 PM
 */

#include "Vector.h"
#include "Math.h"

namespace LSE {
    namespace Util{
        namespace Math{
            
            template<int _N>
            Vector<_N>::Vector() {
            }
            
            template<int _N>
            Vector<_N>::Vector(const float& f) {
                for (int i = 0; i < _N; i++)
                    data[i] = f;
            }
            
            
            template<int _N>
            Vector<_N>::Vector(Vector<_N - 1 > orig) {
                memcpy(data, orig.data, sizeof (orig.data));
                data[_N - 1] = 1.0;
            }
            
            template<int _N>
            Vector<_N>::Vector(Vector<_N + 1 > orig) {
                for (int i = 0; i < _N; i++) {
                    data[i] = orig.data[i] / orig.data[_N];
                }
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::clamp(float m, float M) {
                Vector<_N> ret;
                for (int i = 0; i < _N; i++) {
                    ret.data[i] = clampf(data[i], m, M);
                }
                return ret;
            }
            template<int _N>
            Vector<_N> Vector<_N>::clampLengthUpTo(double maxLen) {
                double len2=length2();
                if(len2>maxLen*maxLen){
                    double len=sqrt(len2);
                    Vector<_N> ret=((*this)*(maxLen/len));
                    return ret;
                }
                return *this;
            }
            
            template<int _N>
            float Vector<_N>::dot(const Vector<_N>& v) {
                float res = 0;
                for (int i = 0; i < _N; i++)
                    res += data[i] * v[i];
                return res;
            }
            
            template<int _N>
            float Vector<_N>::length() const {
                return sqrt(length2());
            }
            
            template<int _N>
            float Vector<_N>::length2() const {
                float res = 0;
                for (int i = 0; i < _N; i++)
                    res += data[i] * data[i];
                return res;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::getNormalized() {
                if (length2() > 0) {
                    return (*this) / length();
                }
                return Vector<_N>(0);
            }
            
            template<int _N>
            bool Vector<_N>::operator!=(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++) {
                    if (fabs(data[i] - v[i]) > .00001)
                        return true;
                }
                return false;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator*(const float& f) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] * f;
                return res;
            }
            
            template<int _N>
            void Vector<_N>::operator*=(const float& f) {
                for (int i = 0; i < _N; i++)
                    data[i] *= f;
            }
            
            template<int _N>
            void Vector<_N>::operator*=(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++)
                    data[i] *= v[i];
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator+(const float& f) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] + f;
                return res;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator+(const Vector<_N>& v) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] + v[i];
                return res;
            }
            
            template<int _N>
            void Vector<_N>::operator+=(const float& f) {
                for (int i = 0; i < _N; i++)
                    data[i] += f;
            }
            
            template<int _N>
            void Vector<_N>::operator+=(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++)
                    data[i] += v[i];
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator-(const float& f) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] - f;
                return res;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator-(const Vector<_N>& v) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] - v[i];
                return res;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator*(const Vector<_N>& v)
            {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] * v[i];
                return res;
            }
            
            template<int _N>
            void Vector<_N>::operator-=(const float& f) {
                for (int i = 0; i < _N; i++)
                    data[i] -= f;
            }
            
            template<int _N>
            void Vector<_N>::operator-=(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++)
                    data[i] -= v[i];
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator/(const float& f) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] / f;
                return res;
            }
            
            template<int _N>
            Vector<_N> Vector<_N>::operator/(const Vector<_N>& v) {
                Vector<_N> res;
                for (int i = 0; i < _N; i++)
                    res[i] = data[i] / v[i];
                return res;
            }
            
            template<int _N>
            void Vector<_N>::operator/=(const float& f) {
                for (int i = 0; i < _N; i++)
                    data[i] /= f;
            }
            
            template<int _N>
            void Vector<_N>::operator/=(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++)
                    data[i] /= v[i];
            }
            
            template<int _N>
            bool Vector<_N>::operator==(const Vector<_N>& v) {
                for (int i = 0; i < _N; i++) {
                    if (fabs(data[i] - v[i]) > .00001)
                        return false;
                }
                return true;
            }
            
            template<int _N>
            Vector<3> Vector<_N>::operator ^(const Vector<3>& v) {
                return Vector < 3 > (data[1] * v[2] - data[2] * v[1], data[2] * v[0] - data[0] * v[2], data[0] * v[1] - data[1] * v[0]);
            }
            
            template<int _N>
            template<int _M>
            Vector<_M> Vector<_N>::toVector() {
                Vector<_M> res;
                for (int i = 0; i < std::min(_N, _M); i++)
                    res[i] = data[i];
                return res;
            }
            
            template<int _N>
            Vector<3> Vector<_N>::fromHexColor(std::string hex) {
                float r = (hex[0] - '0')*16 + (hex[1] - '0');
                float g = (hex[2] - '0')*16 + (hex[3] - '0');
                float b = (hex[4] - '0')*16 + (hex[5] - '0');
                Vector<3> res(r, g, b);
                return res / 255.0;
            }
            
            template<int _N>
            Vector<1> Vector<_N>::x()
            {
                return Vector<1>(data[0]);
            }
            
            template<int _N>
            Vector<2> Vector<_N>::xy()
            {
                return Vector<2>(data[0],data[1]);
            }
            
            template<int _N>
            Vector<3> Vector<_N>::xyz()
            {
                return Vector<3>(data[0],data[1],data[2]);
            }
            
            template class Vector<0>;
            template class Vector<1>;
            template class Vector<2>;
            template class Vector<3>;
            template class Vector<4>;
            template class Vector<5>;
            template class Vector<6>;
        }
    }
}