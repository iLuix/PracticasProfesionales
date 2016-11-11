/*
 * File:   Matrix.cpp
 * Author: Luis
 *
 * Created on 19 de febrero de 2016, 06:52 PM
 */

//#include "Matrix.h"
//#include "Quaternion.h"
#include <cmath>
#include <utility>

namespace LSE {
    namespace Util {
        namespace Math {
            
            double sign(double num) {
                if (num < 0.0) {
                    return -1.0;
                } else if (num > 0.0) {
                    return 1.0;
                } else {
                    return 1.0;
                }
            }
            
            
        }
    }
}
