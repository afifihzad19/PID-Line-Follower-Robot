#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

// PID constants
#define KP 0.8
#define KI 0.0
#define KD 0.1

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Threshold values
#define BLACK_THRESHOLD 300 // Adjust this value based on calibration

int main() {
  wb_robot_init();

  // Initialize sensors
  WbDeviceTag ds_left = wb_robot_get_device("ds_left");
  WbDeviceTag ds_center = wb_robot_get_device("ds_center");
  WbDeviceTag ds_right = wb_robot_get_device("ds_right");

  // Enable sensors
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_center, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // PID control variables
  double integral = 0.0;
  double previous_error = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Read sensor values
    double left_val = wb_distance_sensor_get_value(ds_left);
    double center_val = wb_distance_sensor_get_value(ds_center);
    double right_val = wb_distance_sensor_get_value(ds_right);

    // Print sensor values for debugging
    printf("Left: %f, Center: %f, Right: %f\n", left_val, center_val, right_val);

    // Define error based on sensor values
    double error = 0.0;

    if (center_val < BLACK_THRESHOLD) { // Center sensor on the line
      error = 0;
    } else if (left_val < BLACK_THRESHOLD) { // Left sensor on the line
      error = -1;
    } else if (right_val < BLACK_THRESHOLD) { // Right sensor on the line
      error = 1;
    } else {
      error = previous_error; // Continue with previous error if no line is detected
    }

    // Print error for debugging
    printf("Error: %f\n", error);

    // PID calculations
    integral += error * TIME_STEP;
    double derivative = (error - previous_error) / TIME_STEP;
    double output = KP * error + KI * integral + KD * derivative;
    previous_error = error;

    // Set motor speeds based on PID output
    double left_speed = MAX_SPEED - output;
    double right_speed = MAX_SPEED + output;

    // Clamp speeds to max motor speed
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    // Print motor speeds for debugging
    printf("Left Speed: %f, Right Speed: %f\n", left_speed, right_speed);

    // Apply motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
