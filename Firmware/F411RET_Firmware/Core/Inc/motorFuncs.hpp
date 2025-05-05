/*
 * motorFuncs.hpp
 *
 *  Created on: Apr 14, 2025
 *      Author: saar
 */

#ifndef INC_MOTORFUNCS_HPP_
#define INC_MOTORFUNCS_HPP_

#ifdef __cplusplus
#include <utility>  // Include only for C++ code

#endif

#ifdef __cplusplus
extern "C" {
#endif

// Declare C functions (do not use C++ features here)
void moveMotors(int anglesRec[4]);
void setServoToInit();
void testFunc();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Declare C++ functions here (don't use extern "C")

#endif



#endif /* INC_MOTORFUNCS_HPP_ */
