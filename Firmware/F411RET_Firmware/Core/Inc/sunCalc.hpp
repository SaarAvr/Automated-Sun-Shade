/*
 * sunCalc.hpp
 *
 *  Created on: Apr 10, 2025
 *      Author: saar
 */

#ifndef INC_SUNCALC_HPP_
#define INC_SUNCALC_HPP_

#ifdef __cplusplus
#include <utility>  // Include only for C++ code

#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "solTrack.h"

typedef struct {
    double azimuth;
    double altitude;
} sunPos;

sunPos sunTest (int year, int month, int day, int hour, int minute, double lat, double lon);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Declare C++ functions here (don't use extern "C")
//std::pair<double, double> sum(int x, int y);
#endif


#endif /* INC_SUNCALC_HPP_ */
