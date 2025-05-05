/*
 * calculations.hpp
 *
 *  Created on: Feb 4, 2025
 *      Author: saar
 */



//#ifndef INC_CALCULATIONS_HPP_
//#define INC_CALCULATIONS_HPP_
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include <utility>
//
//std::pair<double,double> sum(int,int);
//
//
//
//
//#ifdef __cplusplus
//}
//#endif
//
//
//
//#endif /* INC_CALCULATIONS_HPP_ */



//--------

//#ifndef CALCULATIONS_HPP
//#define CALCULATIONS_HPP
//#ifndef INC_CALCULATIONS_HPP_
//#define INC_CALCULATIONS_HPP_
//
//#ifdef __cplusplus
//// Only use the C++ standard library features in C++ code.
//#include <utility>
//
//// Declare your C++ functions or classes here
//int pairSum(int x, int y);
//std::pair<double, double> sum(int x, int y);
//int regularSum(int x, int y);
//
//
//#endif  // __cplusplus
//
//#endif  // CALCULATIONS_HPP


//--------------------------------------

#ifndef INC_CALCULATIONS_HPP_
#define INC_CALCULATIONS_HPP_

#ifdef __cplusplus
#include <utility>  // Include only for C++ code
#include "solTrack.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Structure to hold the 4 integers
typedef struct {
    int val1;
    int val2;
    int val3;
    int val4;
} motorAngles;


// Declare C functions (do not use C++ features here)
int pairSum(int x, int y);
int regularSum(int x, int y);
motorAngles anglesCalculation(double azi, double alt);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Declare C++ functions here (don't use extern "C")
std::pair<double, double> sum(int x, int y);
#endif

#endif  // CALCULATIONS_HPP_









