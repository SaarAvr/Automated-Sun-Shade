import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import numpy as np
import time
import serial
import threading
from sun_position_calculator import SunPositionCalculator
import struct

arduino_message = None
comm_count = 1

# def send_four_integers(angles):
#     # Pack the 4 integers into a binary format (4 x 4 bytes = 16 bytes total)
#     message = struct.pack('<iiii', angles[0], angles[1], angles[2], angles[3])
#     ser.write(message)  # Send the binary message

def send_four_integers(angles):
    message = struct.pack('<iiii', angles[0], angles[1], angles[2], angles[3])
    # ser.write(b'\xFF' + message + b'\xFE')  # Add start (0xFF) and end (0xFE) markers
    time.sleep(0.1)

def send_motor_angles(angles):
    message = ",".join(map(str, angles)) + "\n"  # Format the message
    print(f"sending: {message}")
    # ser.write(message.encode())  # Send message

def listen_to_arduino():
    global arduino_message
    while True:
        temp = 5;
        # if ser.in_waiting > 0:  # Check if there are bytes to read
        #     breakpoint()
        #     arduino_message = ser.readline().decode().strip()  # Read message from Arduino
        #     print("\n" + "=" * 30)  # Print a separator line
        #     print(f"Arduino: {arduino_message}")  # Print the received message
        #     print("=" * 30 + "\n")  # Print another separator line
def calcErrors(sun, continuation_point):
    closest_point = closest_point_on_line(shade_point, sun, continuation_point)
    closest_pointXY = (closest_point[1], closest_point[0])
    stick2_XY = (continuation_point[1], continuation_point[0])
    stick2_xyDeg = np.degrees(np.arctan2(continuation_point[1], continuation_point[0]))
    closest_point_xyDeg = np.degrees(np.arctan2(closest_point[1], closest_point[0]))
    closest_point_zDeg = np.degrees(np.arctan2(closest_point[2], np.linalg.norm(closest_pointXY)))
    stick2_deg = np.degrees(np.arctan2(continuation_point[2], np.linalg.norm(stick2_XY)))

    if (closest_point_xyDeg < 0):
        closest_point_xyDeg += 360
    if (stick2_xyDeg < 0):
        stick2_xyDeg += 360
    motor1_err = closest_point_xyDeg - stick2_xyDeg
    if (motor1_err > 330):
        motor1_err = -10
    elif (motor1_err < -330):
        motor1_err = 10
    # motor2_err = closest_point_zDeg - motor_angle2
    motor2_err = closest_point_zDeg - stick2_deg
    # print(f"motor1_err: {motor1_err}")
    # print(f"motor2_err: {motor2_err}")

    return motor1_err,motor2_err

def closest_point_on_line(line_point1, line_point2, point):
    # Convert points to numpy arrays for easier vector operations
    line_point1 = np.array(line_point1)
    line_point2 = np.array(line_point2)
    point = np.array(point)

    # Calculate the direction vector of the line
    line_direction = line_point2 - line_point1

    # Calculate the vector between line_point1 and the given point
    vector_to_point = point - line_point1

    # Calculate the projection of vector_to_point onto line_direction
    # print(f"line direction = {line_direction}")
    # print(f"np.dot up = {np.dot(vector_to_point, line_direction)}")
    # print(f"np.dot down = {np.dot(line_direction, line_direction)}")
    projection = np.dot(vector_to_point, line_direction) / np.dot(line_direction, line_direction)

    # Calculate the closest point on the line
    closest_point = line_point1 + projection * line_direction

    return closest_point

def spherical_to_cartesian(azimuth, elevation, distance):
    # Convert spherical coordinates to Cartesian coordinates
    x = distance * np.sin(elevation) * np.cos(azimuth)
    y = distance * np.sin(elevation) * np.sin(azimuth)
    z = distance * np.cos(elevation)
    return x, y, z
def rotate_point_new(point, angle, axis):
    """
    Rotate a point around a given axis by a specific angle.

    :param point: 3D vector to rotate
    :param angle: Rotation angle in degrees
    :param axis: Axis to rotate around (a 3D vector)
    :return: Rotated 3D point
    """
    # Convert the angle to radians
    angle_rad = np.radians(angle)

    # Normalize the axis vector
    axis = axis / np.linalg.norm(axis)

    # Rodrigues' rotation formula for rotating around an arbitrary axis
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    cross_prod_matrix = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    rotation_matrix = (
            cos_angle * np.eye(3) +
            sin_angle * cross_prod_matrix +
            (1 - cos_angle) * np.outer(axis, axis)
    )

    # Apply the rotation matrix to the point
    rotated_point = np.dot(rotation_matrix, point)

    return rotated_point

# Function to rotate a point around an arbitrary axis
def rotate_point(point, angle_degrees, axis='z'):
    angle_radians = np.radians(angle_degrees)
    if axis == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(angle_radians), -np.sin(angle_radians)],
                                    [0, np.sin(angle_radians), np.cos(angle_radians)]])
    elif axis == 'y':
        rotation_matrix = np.array([[np.cos(angle_radians), 0, np.sin(angle_radians)],
                                    [0, 1, 0],
                                    [-np.sin(angle_radians), 0, np.cos(angle_radians)]])
    elif axis == 'z':
        rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                                    [np.sin(angle_radians), np.cos(angle_radians), 0],
                                    [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    return np.dot(rotation_matrix, point)

# Function to compute the sun vector based on latitude and longitude
def compute_sun_vector(latitude, longitude):
    lat_rad = np.radians(latitude)
    lon_rad = np.radians(longitude)
    return np.array([
        sun_dist * np.cos(lat_rad) * np.cos(lon_rad),  # X component
        sun_dist * np.cos(lat_rad) * np.sin(lon_rad),  # Y component
        sun_dist * np.sin(lat_rad)                    # Z component
    ])

# Function to calculate pan and tilt angles to align the continuation vector with the sun vector
def calculate_pan_tilt_angles(current_vector, sun_vector):
    # Normalize the vectors
    sun_vector = sun_vector / np.linalg.norm(sun_vector)
    current_vector = current_vector / np.linalg.norm(current_vector)

    # Step 1: Define the local Z-axis as the current vector (stick vector)
    local_z = current_vector

    # Step 2: Find the local X-axis using ang1 and ang2 for correct rotation
    local_x = np.array([1, 0, 0])  # Default X-axis
    local_x = rotate_point(local_x, ang2, axis='x')
    local_x = rotate_point(local_x, ang1, axis='z')

    # Step 3: Calculate the local Y-axis, perpendicular to both local X and local Z
    local_y = np.cross(local_z, local_x)
    local_y /= np.linalg.norm(local_y)  # Normalize the local Y-axis

    # Print local axes for debugging
    # print(f"local X in func: {local_x}")
    # print(f"local Y in func: {local_y}")
    # print(f"local Z in func: {local_z}")

    # Construct the rotation matrix (from global to local coordinate system)
    rotation_matrix = np.column_stack((local_x, local_y, local_z))

    # Step 4: Transform the sun vector into the local coordinate system
    sun_in_local = np.dot(np.linalg.inv(rotation_matrix), sun_vector)

    # Print the transformed sun vector in the local coordinate system
    # print(f"Sun vector in local coordinates: {sun_in_local}")

    # Step 5: Project the sun vector onto the local XY plane (zero out Z component)
    sun_xy = np.array([sun_in_local[0], sun_in_local[1], 0])
    sun_xy /= np.linalg.norm(sun_xy)  # Normalize the projected sun vector

    # Step 6: Calculate the pan angle as the angle between local -Y and the projected sun
    south_direction = np.array([0, -1, 0])  # Local -Y direction
    dot_product = np.dot(south_direction, sun_xy)  # Dot product with local -Y
    pan_angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Calculate angle

    # Step 7: Determine the sign of the pan angle using cross product
    cross_product = np.cross(south_direction, sun_xy)
    if cross_product[2] < 0:
        pan_angle = -pan_angle

    # Step 8: Calculate the tilt angle (angle between sun and local Z-axis)
    tilt_angle = np.arccos(np.clip(np.dot(sun_in_local, [0, 0, 1]), -1.0, 1.0))

    # Convert to degrees for motor control
    pan_angle_degrees = np.degrees(pan_angle)
    tilt_angle_degrees = np.degrees(tilt_angle)

    if (pan_angle_degrees < -90):
        pan_angle_degrees += 180
        tilt_angle_degrees = -tilt_angle_degrees
    if (pan_angle_degrees > 90):
        pan_angle_degrees -= 180
        tilt_angle_degrees = -tilt_angle_degrees

    # # Debugging: Check pan and tilt angle calculations
    # print(f"Calculated Pan Angle (degrees): {pan_angle_degrees}")
    # print(f"Calculated Tilt Angle (degrees): {tilt_angle_degrees}")

    return pan_angle_degrees, tilt_angle_degrees



# Initialize serial connection (adjust port as needed)
# ser = serial.Serial('/dev/cu.usbmodem144101', 9600, timeout=1)
time.sleep(2)  # Allow time for the Arduino to reset

# # Create and start a thread for listening to Arduino messages
listen_thread = threading.Thread(target=listen_to_arduino)
listen_thread.daemon = True  # Daemon thread ends when the main thread ends
listen_thread.start()

# Create a larger figure window
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([0, 4])

# Plot original XYZ axes
ax.quiver(0, 0, 0, 1, 0, 0, color='red', arrow_length_ratio=0.1)  # X-axis
ax.quiver(0, 0, 0, 0, 1, 0, color='green', arrow_length_ratio=0.1)  # Y-axis
ax.quiver(0, 0, 0, 0, 0, 1, color='blue', arrow_length_ratio=0.1)  # Z-axis

# Tel Aviv location
latitude = 32.109333
longitude = 34.855499

unix_test = int(time.time() * 1000)
print(f"time is: {unix_test}")

# Create an instance of the SunPositionCalculator
calculator = SunPositionCalculator()

# Calculate the sun position
position = calculator.pos(unix_test, latitude, longitude)
azimuth = np.degrees(position.azimuth)
altitude = np.degrees(position.altitude)

sun_azimuth = 90
sun_altitude = 40
ang1 = 0  # Rotation around Z-axis
ang2 = 45  # Rotation around X-axis

unit_size = 1/10
initial_vector = np.array([0, 0, 9*unit_size])
initial_vec_err = np.array([0,-1.2*unit_size,0])
stick_position = np.array([0, 0, 6*unit_size])


vector_x_rotated = rotate_point(initial_vector, ang2, 'x')
err_vec_rotated = rotate_point(initial_vec_err, ang1, 'z')
final_vector = rotate_point(vector_x_rotated, ang1, 'z')
continuation_point = stick_position + final_vector + err_vec_rotated

print(f"continuation_point1: {continuation_point}")

shade_point = (0, 6*unit_size, 0)
sun_dist = 3
# sun = (np.radians(90 - sun_azimuth), np.radians(90 - sun_altitude), sun_dist)
# sun = spherical_to_cartesian(sun[0], sun[1], sun[2])
# sun = (sun[0] + shade_point[0], sun[1] + shade_point[1], sun[2] + shade_point[2])

ang1Max,ang2Max,panMax,tiltMax = 0,0,0,0
ang1Min,ang2Min,panMin,tiltMin = 0,0,0,0

def update_plot(val):
    global ang1,ang2,continuation_point,ang1Min,ang2Min,panMin,tiltMin,ang1Max,ang2Max,panMax,tiltMax,comm_count
    lat = latitude_slider.val
    lon = longitude_slider.val

    # # Calculate the sun position
    # position = calculator.pos(unix_test, latitude, longitude)
    # lon = np.degrees(position.azimuth)
    # lat = np.degrees(position.altitude)

    sun = compute_sun_vector(lat,90 - lon)
    sun = sun + shade_point

    sun_vector = compute_sun_vector(lat,90 - lon)
    current_vector = continuation_point - stick_position
    # print(f"continuation_point2: {continuation_point}")
    # print(f"sun2: {sun}")
    closest_point = closest_point_on_line(shade_point, sun, continuation_point)
    # print(f"closest_point: {closest_point}")

    error_dist = 1
    counter = 0
    while (error_dist > 0.05):

        counter = counter + 1
        if (counter == 200):
            break
        # time.sleep(0.1)

        print(f"{counter}-----------------------------------------------------")

        motor1_err, motor2_err = calcErrors(sun, continuation_point)

        if motor1_err > 5:
            ang1 += 5
        elif motor1_err > 1:
            ang1 += 1
        elif motor1_err < -5:
            ang1 -= 5
        elif motor1_err < -1:
            ang1 -= 1

        if motor1_err < 10 and motor1_err > -10:
            if motor2_err > 1:
                ang2 -= 1
            elif motor2_err < -1:
                ang2 += 1

        if ang1 < -360:
            ang1 += 360
        if ang2 < -360:
            ang2 += 360
        if ang1 > 360:
            ang1 -= 360
        if ang2 > 360:
            ang2 -= 360

        vector_x_rotated = rotate_point(initial_vector, ang2, 'x')
        err_vec_rotated = rotate_point(initial_vec_err, ang1, 'z')
        final_vector = rotate_point(vector_x_rotated, ang1, 'z')
        continuation_point = stick_position + final_vector + err_vec_rotated
        closest_point = closest_point_on_line(shade_point, sun, continuation_point)

        error_dist = np.linalg.norm(np.array(closest_point) - np.array(continuation_point))
        print(f"motor1_err: {motor1_err}")
        print(f"motor2_err: {motor2_err}")
        print(f"error_dist: {error_dist}")

    current_vector = continuation_point - (err_vec_rotated + stick_position)

    pan_angle, tilt_angle = calculate_pan_tilt_angles(current_vector, sun_vector)

    # sending arduino the calculated angles
    angles = [int(ang1), int(ang2), int(pan_angle), int(tilt_angle)]
    print(f"{comm_count} >>>>>>>>>>>>>>> sending: {angles}")
    comm_count = comm_count + 1
    send_four_integers(angles)


    if ang1 < ang1Min:
        ang1Min = ang1
    if ang2 < ang2Min:
        ang2Min = ang2
    if pan_angle < panMin:
        panMin = pan_angle
    if tilt_angle < tiltMin:
        tiltMin = tilt_angle
    if ang1 > ang1Max:
        ang1Max = ang1
    if ang2 > ang2Max:
        ang2Max = ang2
    if pan_angle > panMax:
        panMax = pan_angle
    if tilt_angle > tiltMax:
        tiltMax = tilt_angle


    # Compute outgoing vector aligned with sun vector
    outgoing_vector = np.array([0, 0, 0.5])  # Initial outgoing vector
    outgoing_vector = rotate_point(outgoing_vector, ang2, 'x')
    outgoing_vector = rotate_point(outgoing_vector, ang1, 'z')

    # Step 1: Compute local Z axis
    local_z = outgoing_vector / np.linalg.norm(outgoing_vector)

    # Step 2: Compute local X axis
    local_x = np.cross(np.array([0, 0, 1]), local_z)
    if np.linalg.norm(local_x) == 0:  # Handle case where vectors are collinear
        local_x = np.array([1, 0, 0])  # Default local X axis

    local_x = local_x / np.linalg.norm(local_x)

    # Step 3: Compute local Y axis (optional, but can be useful for future purposes)
    local_y = np.cross(local_z, local_x)

    # # Print local axes for debugging
    # print(f"Local X axis: {local_x}")
    # print(f"Local Y axis: {local_y}")
    # print(f"Local Z axis: {local_z}")

    # Step 4: Apply tilt and pan rotations using the local axes
    outgoing_vector = rotate_point_new(outgoing_vector, tilt_angle, local_x)  # Rotate around local X-axis by tilt_angle
    outgoing_vector = rotate_point_new(outgoing_vector, pan_angle, local_z)  # Rotate around local Z-axis by pan_angle

    # print(f"outgoing_vector: {outgoing_vector}")
    #
    # print(f"ang1: {ang1}")
    # print(f"ang2: {ang2}")
    # print(f"Calculated Pan Angle: {pan_angle}")
    # print(f"Calculated Tilt Angle: {tilt_angle}")
    # print(f"ang1 Range: {ang1Min} : {ang1Max}")
    # print(f"ang2 Range: {ang2Min} : {ang2Max}")
    # print(f"pan Range: {panMin} : {panMax}")
    # print(f"tilt Range: {tiltMin} : {tiltMax}")

    parallel_vector_endpoint = continuation_point + outgoing_vector

    ax.cla()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([0, 4])

    # Plot XYZ axes
    ax.quiver(0, 0, 0, 0.5, 0, 0, color='red', arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0.5, 0, color='green', arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0, 0.5, color='blue', arrow_length_ratio=0.1)

    # Plot stick (line from origin to stick)
    ax.plot([0, stick_position[0]], [0, stick_position[1]], [0, stick_position[2]], color='orange', linewidth=3, label='Base')

    err_vec_rotated = rotate_point(initial_vec_err, ang1, 'z')
    correction = stick_position + err_vec_rotated
    # Plot correction vector
    ax.plot([stick_position[0], correction[0]], [stick_position[1], correction[1]], [stick_position[2], correction[2]],
            color='firebrick', linewidth=3, label='Base arm')

    # Plot continuation point (line from stick to continuation point)
    ax.plot([correction[0], continuation_point[0]], [correction[1], continuation_point[1]], [correction[2], continuation_point[2]],
            color='magenta', linewidth=3, label='Main arm')

    # Plot outgoing vector (line from continuation point to parallel vector endpoint)
    ax.plot([continuation_point[0], parallel_vector_endpoint[0]],
            [continuation_point[1], parallel_vector_endpoint[1]],
            [continuation_point[2], parallel_vector_endpoint[2]],
            color='purple', linewidth=7, label='Parasol')

    # # Plot sun vector
    # ax.plot([0, sun_vector[0]], [0, sun_vector[1]], [0, sun_vector[2]],
    #         color='cyan', linewidth=2, label='Sun Vector')
    ax.plot([shade_point[0], sun[0]], [shade_point[1], sun[1]], [shade_point[2], sun[2]],
            color='yellow', linewidth=2, linestyle='--', label='Sun Vector')
    ax.scatter(sun[0], sun[1], sun[2], c='orange', marker='o', s=100)  # adding sun
    ax.scatter(closest_point[0], closest_point[1], closest_point[2], c='blue', marker='o', s=50)  # adding closest point

    ax.legend()
    plt.draw()

# Add sliders for latitude and longitude
ax_latitude = plt.axes([0.1, 0.02, 0.65, 0.03])
latitude_slider = Slider(ax_latitude, 'Latitude', 0, 90, valinit=sun_altitude)

ax_longitude = plt.axes([0.1, 0.06, 0.65, 0.03])
longitude_slider = Slider(ax_longitude, 'Longitude', 0, 360, valinit=sun_azimuth)


latitude_slider.on_changed(update_plot)
longitude_slider.on_changed(update_plot)

# Initial plot
update_plot(None)
plt.show()
