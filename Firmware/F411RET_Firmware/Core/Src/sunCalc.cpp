/*
 * sunCalc.cpp
 *
 *  Created on: Apr 10, 2025
 *      Author: saar
 */

#include "sunCalc.hpp"
#include "SolTrack.h"
#include <math.h>

sunPos sunTest(int year, int month, int day, int hour, int minute, double lat, double lon){
	  sunPos sunPosition;
	  int useDegrees = 1;             // Input (geographic position) and output are in degrees
	  int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
	  int computeRefrEquatorial = 0;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
	  int computeDistance = 1;        // Compute the distance to the Sun in AU: 0-no, 1-yes


	  struct Time time;

	  // Set (UT!) date and time manually - use the first date from SolTrack_positions.dat:
	  time.year = year;
	  time.month = month;
	  time.day = day;
	  time.hour = hour - 3;  // HAS TO BE UT
	  time.minute = minute;
	  time.second = 0;

	  struct Location loc;
	  loc.latitude  = lat;
	  loc.longitude = lon;
	  loc.pressure = 101.0;     // Atmospheric pressure in kPa
	  loc.temperature = 283.0;  // Atmospheric temperature in K


	  // Compute rise and set times:
	  struct Position pos;
	  struct RiseSet riseSet;
	  SolTrack_RiseSet(time, loc, &pos, &riseSet, 0.0, useDegrees, useNorthEqualsZero);


	  // Compute positions:
	  SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

//	  printf("azimuth: %.2f, altitude: %.2f\r\n",pos.azimuthRefract, pos.altitude);

	  sunPosition.azimuth = pos.azimuthRefract;
	  sunPosition.altitude = pos.altitude;

	  return sunPosition;
}
