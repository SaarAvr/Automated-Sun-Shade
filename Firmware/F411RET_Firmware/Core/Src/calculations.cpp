/*
 * calculations.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: saar
 */

#include "calculations.hpp"
#include <math.h>
#include <iostream>


std::pair<double, double> sum(int x, int y) {
    double m, n;
    m = x;
    n = y;

    // Create a pair and assign values
    std::pair<double, double> pair1;
    pair1.first = m;
    pair1.second = n;

    // Return the pair
//    int res = pairSum(pair1);
    return pair1;
}

int pairSum (int x, int y){
	std::pair<double,double> summation = sum(x, y);
	return summation.first + summation.second;
}

int regularSum(int x, int y){
	return x + y;
}


// Define a struct to represent a 3D point
struct point {
  double x;
  double y;
  double z;
};

// Variables
double ang1 = 0;  // Rotation around Z-axis
double ang2 = 45;  // Rotation around X-axis

double unit_size = 1.0 / 10.0;

// Vectors
point initial_vector = {0, 0, 9 * unit_size};
point initial_vec_err = {0, -1.2 * unit_size, 0};
point stick_position = {0, 0, 6 * unit_size};

point testVector = {0,0,1};

// Shade point and sun distance
point shade_point = {0, 6 * unit_size, 0};
double sun_dist = 3;

double degrees(double radians) {
    return radians * (180.0 / M_PI);
}

point sum_points(point p1, point p2) {
  point result;
  result.x = p1.x + p2.x;  // Sum of x components
  result.y = p1.y + p2.y;  // Sum of y components
  result.z = p1.z + p2.z;  // Sum of z components
  return result;
}

point subtract_points(point p1, point p2) {
  point result;
  result.x = p1.x - p2.x;
  result.y = p1.y - p2.y;
  result.z = p1.z - p2.z;
  return result;
}

double dot_product(point p1, point p2) {
  return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

point scale_point(point p, double scalar) {
  point result;
  result.x = p.x * scalar;
  result.y = p.y * scalar;
  result.z = p.z * scalar;
  return result;
}

point normalize(point vec) {
  double norm = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
  return {vec.x / norm, vec.y / norm, vec.z / norm};
}

point cross_product(point a, point b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

point rotate_point(point p, double angle_degrees, char axis) {
  // Convert angle from degrees to radians
  double angle_radians = angle_degrees * (M_PI / 180.0);

  // Declare the rotation matrix and the result point
  double rotation_matrix[3][3];
  point result;

  // Set up the rotation matrix based on the chosen axis
  if (axis == 'x') {
    rotation_matrix[0][0] = 1; rotation_matrix[0][1] = 0; rotation_matrix[0][2] = 0;
    rotation_matrix[1][0] = 0; rotation_matrix[1][1] = cos(angle_radians); rotation_matrix[1][2] = -sin(angle_radians);
    rotation_matrix[2][0] = 0; rotation_matrix[2][1] = sin(angle_radians); rotation_matrix[2][2] = cos(angle_radians);
  }
  else if (axis == 'y') {
    rotation_matrix[0][0] = cos(angle_radians); rotation_matrix[0][1] = 0; rotation_matrix[0][2] = sin(angle_radians);
    rotation_matrix[1][0] = 0; rotation_matrix[1][1] = 1; rotation_matrix[1][2] = 0;
    rotation_matrix[2][0] = -sin(angle_radians); rotation_matrix[2][1] = 0; rotation_matrix[2][2] = cos(angle_radians);
  }
  else if (axis == 'z') {
    rotation_matrix[0][0] = cos(angle_radians); rotation_matrix[0][1] = -sin(angle_radians); rotation_matrix[0][2] = 0;
    rotation_matrix[1][0] = sin(angle_radians); rotation_matrix[1][1] = cos(angle_radians); rotation_matrix[1][2] = 0;
    rotation_matrix[2][0] = 0; rotation_matrix[2][1] = 0; rotation_matrix[2][2] = 1;
  }
  else {
    // Handle invalid axis input (Optional, as Arduino might not use exceptions)
//    Serial.println("Invalid axis. Use 'x', 'y', or 'z'.");
    return result;  // Return an unmodified result in case of invalid axis
  }

  // Perform the matrix multiplication to get the rotated point
  result.x = rotation_matrix[0][0] * p.x + rotation_matrix[0][1] * p.y + rotation_matrix[0][2] * p.z;
  result.y = rotation_matrix[1][0] * p.x + rotation_matrix[1][1] * p.y + rotation_matrix[1][2] * p.z;
  result.z = rotation_matrix[2][0] * p.x + rotation_matrix[2][1] * p.y + rotation_matrix[2][2] * p.z;

  return result;  // Return the rotated point
}

// Function to calculate pan and tilt angles
std::pair<double, double> calculate_pan_tilt_angles(point current_vector, point sun_vector, double ang1, double ang2) {
  // Normalize vectors
  sun_vector = normalize(sun_vector);
  current_vector = normalize(current_vector);

  // Step 1: Define the local Z-axis
  point local_z = current_vector;

  // Step 2: Calculate local X-axis
  point local_x = {1, 0, 0}; // Default X-axis
  local_x = rotate_point(local_x, ang2, 'x');
  local_x = rotate_point(local_x, ang1, 'z');

  // Step 3: Calculate local Y-axis
  point local_y = cross_product(local_z, local_x);
  local_y = normalize(local_y);

  // Step 4: Construct the rotation matrix
  double rotation_matrix[3][3] = {
    {local_x.x, local_y.x, local_z.x},
    {local_x.y, local_y.y, local_z.y},
    {local_x.z, local_y.z, local_z.z}
  };

  // Step 5: Transform the sun vector into the local coordinate system
  point sun_in_local = {
    dot_product(sun_vector, {rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0]}),
    dot_product(sun_vector, {rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1]}),
    dot_product(sun_vector, {rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2]})
  };

  // Step 6: Project sun vector onto the local XY plane
  point sun_xy = {sun_in_local.x, sun_in_local.y, 0};
  sun_xy = normalize(sun_xy);

  // Step 7: Calculate the pan angle
  point south_direction = {0, -1, 0};
  double dot = dot_product(south_direction, sun_xy);
  double pan_angle = acos(fmin(fmax(dot, -1.0), 1.0)) * 180.0 / M_PI;

  point cross = cross_product(south_direction, sun_xy);
  if (cross.z < 0) {
    pan_angle = -pan_angle;
  }

  // Step 8: Calculate the tilt angle
  double tilt_angle = acos(fmin(fmax(dot_product(sun_in_local, {0, 0, 1}), -1.0), 1.0)) * 180.0 / M_PI;

  // Step 9: Adjust angles to motor limits
  if (pan_angle < -90) {
    pan_angle += 180;
    tilt_angle = -tilt_angle;
  }
  if (pan_angle > 90) {
    pan_angle -= 180;
    tilt_angle = -tilt_angle;
  }

  // Return pan and tilt angles
  return std::pair <double, double> {pan_angle, tilt_angle};
}

// Function to compute the closest point on a line defined by two points (line_point1, line_point2) from a third point
point closest_point_on_line(point line_point1, point line_point2, point target) {
  // Calculate the direction vector of the line
  point line_direction = subtract_points(line_point2, line_point1);

  // Calculate the vector between line_point1 and the given point
  point vector_to_point = subtract_points(target, line_point1);

  // Calculate the projection of vector_to_point onto line_direction
  double dot_product_up = dot_product(vector_to_point, line_direction);
  double dot_product_down = dot_product(line_direction, line_direction);
  double projection = dot_product_up / dot_product_down;

  // Calculate the closest point on the line
  point closest_point = sum_points(line_point1, scale_point(line_direction, projection));

  return closest_point;
}

// Function to compute the error in motor positions based on the sun and continuation point
std::pair<double, double> calcErrors(point sun, point continuation_point) {
  // Calculate the closest point on the line between shade_point and sun
  point closest_point = closest_point_on_line(shade_point, sun, continuation_point);

  // Calculate the XY components of the points
  double closest_pointXY[2] = {closest_point.y, closest_point.x};
  double stick2_XY[2] = {continuation_point.y, continuation_point.x};

  // Calculate the angles
  double stick2_xyDeg = degrees(atan2(continuation_point.y, continuation_point.x));
  double closest_point_xyDeg = degrees(atan2(closest_point.y, closest_point.x));
  double closest_point_zDeg = degrees(atan2(closest_point.z, sqrt(pow(closest_pointXY[0], 2) + pow(closest_pointXY[1], 2))));
  double stick2_deg = degrees(atan2(continuation_point.z, sqrt(pow(stick2_XY[0], 2) + pow(stick2_XY[1], 2))));

  // Adjust for negative angles
  if (closest_point_xyDeg < 0) closest_point_xyDeg += 360;
  if (stick2_xyDeg < 0) stick2_xyDeg += 360;

  double motor1_err, motor2_err;
  // Calculate the motor errors
  motor1_err = closest_point_xyDeg - stick2_xyDeg;
  if (motor1_err > 330) motor1_err = -10;
  else if (motor1_err < -330) motor1_err = 10;

  // Motor2 error (difference between closest_point_zDeg and stick2_deg)
  motor2_err = closest_point_zDeg - stick2_deg;
  return std::pair <double,double> {motor1_err, motor2_err};
}



point compute_sun_vector(double latitude, double longitude) {
  // Convert latitude and longitude to radians
  double lat_rad = latitude * (M_PI / 180.0);
  double lon_rad = longitude * (M_PI / 180.0);

  // Compute the components of the sun vector
  point sun_vector;
  sun_vector.x = sun_dist * cos(lat_rad) * cos(lon_rad);  // X component
  sun_vector.y = sun_dist * cos(lat_rad) * sin(lon_rad);  // Y component
  sun_vector.z = sun_dist * sin(lat_rad);                 // Z component

  return sun_vector;
}

motorAngles anglesCalculation(double azi, double alt){
  motorAngles angles;

  double sun_azimuth = azi;
  double sun_altitude = alt;
  point sun = compute_sun_vector(sun_altitude, 90 - sun_azimuth);
  point sunVector = compute_sun_vector(sun_altitude, 90 - sun_azimuth);
  sun = sum_points(sun, shade_point);

  point vector_x_rotated = rotate_point(initial_vector, ang2, 'x');
  point err_vec_rotated = rotate_point(initial_vec_err, ang1, 'z');
  point final_vector = rotate_point(vector_x_rotated, ang1, 'z');
  point continuation_point = sum_points(sum_points(stick_position, final_vector), err_vec_rotated);

  point closest_point = closest_point_on_line(shade_point, sun, continuation_point);

  double error_dist = 1;
  int counter = 0;
  double motor1_err, motor2_err;

  while (error_dist > 0.05){
	counter++;
	if (counter == 200){
//	  Serial.println("too many iterations");
	  break;
	}
	double motor1_err, motor2_err;
	auto result = calcErrors(sun, continuation_point);
	motor1_err = result.first;
	motor2_err = result.second;

	// Adjust ang1 and ang2 based on errors
	if (motor1_err > 5) {
	  ang1 += 5;
	} else if (motor1_err > 1) {
		ang1 += 1;
	} else if (motor1_err < -5) {
		ang1 -= 5;
	} else if (motor1_err < -1) {
		ang1 -= 1;
	}

	if (motor1_err < 10 and motor1_err > -10){
		if (motor2_err > 1) {
		  ang2 -= 1;
		} else if (motor2_err < -1) {
		  ang2 += 1;
		}
	}

	// Normalize angles between -360 and 360
	if (ang1 < -360) ang1 += 360;
	if (ang2 < -360) ang2 += 360;
	if (ang1 > 360) ang1 -= 360;
	if (ang2 > 360) ang2 -= 360;

	// Rotate the vectors based on the angles
	vector_x_rotated = rotate_point(initial_vector, ang2, 'x');
	err_vec_rotated = rotate_point(initial_vec_err, ang1, 'z');
	final_vector = rotate_point(vector_x_rotated, ang1, 'z');
	continuation_point = sum_points(sum_points(stick_position, final_vector), err_vec_rotated);
	closest_point = closest_point_on_line(shade_point, sun, continuation_point);

	// Calculate the error distance
	error_dist = sqrt(pow(closest_point.x - continuation_point.x, 2) + pow(closest_point.y - continuation_point.y, 2) + pow(closest_point.z - continuation_point.z, 2));
  }

  point current_vector = subtract_points(continuation_point, sum_points(err_vec_rotated, stick_position));
  double pan_angle, tilt_angle;
  auto panTiltResult = calculate_pan_tilt_angles(current_vector, sunVector, ang1, ang2);
  pan_angle = panTiltResult.first;
  tilt_angle = panTiltResult.second;

	angles.val1 = ang1;
	angles.val2 = ang2;
	angles.val3 = pan_angle;
	angles.val4 = tilt_angle;
//	printf("finished calculating\r\n");
 	return angles;
}























