/*
 * File:   Math.h
 * Author: Luis
 *
 * Created on 19 de febrero de 2016, 07:14 PM
 */

#ifndef MATH_H
#define	MATH_H

//#include "LSE/PlatformDefines.h"
#include "Vector.h"
#include <cstdint>

namespace LSE {
    namespace Util {
        namespace Math {
            const double Pi = 3.14159265359;
            const double E = 2.7182818284;
            /**
             * Calcula el entero mas cercano a x
             * @param x
             * @return
             */
            double getNearestInteger(double x);
            /**
             * Calcula el par mas cercano a x
             * @param x
             * @return
             */
            double getNearestPair(double x);
            /**
             * Calcula el impar mas cercano a x
             * @param x
             * @return
             */
            double getNearestOdd(double x);
            
            double /*GC_LIBRARY_EXPORT*/ clamp(double value, double m, double M);
            
            inline double lerp(double from, double to, double t) {
                double r;
                r = from + (to - from) * t;
                return r;
            }
            
            inline float clampf(float value, float m, float M) {
                if (value < m) {
                    value = m;
                } else if (value > M) {
                    value = M;
                }
                return value;
            }
            
            inline float lerpf(float from, float to, float t) {
                double r;
                r = from + (to - from) * t;
                return r;
            }
            
            template<int N>
            Vector<N> abs(Vector<N> vec) {
                Vector<N> ret;
                for (int i = 0; i < N; i++) {
                    if (vec.data[i] < 0) {
                        ret.data[i] = -vec.data[i];
                    } else {
                        ret.data[i] = vec.data[i];
                    }
                }
                return ret;
            }
            int gcd(int a, int b);
            int64_t mcm(int a, int b);
            
            template<int N> Vector<N> lerp(Vector<N> &from, Vector<N> &to, float t) {
                Vector<N> ret;
                for (int i = 0; i < N; i++) {
                    ret.data[i] = lerpf(from.data[i], to.data[i], t);
                }
                return ret;
            }
        }
    }
}


#endif	/* MATH_H */