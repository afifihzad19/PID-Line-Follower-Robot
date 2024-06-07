#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 10.0
#define KP 0.1
#define KD 0.01

// Fungsi untuk mengatur kecepatan motor kiri dan kanan
void set_motor_speeds(WbDeviceTag left_motor, WbDeviceTag right_motor, double left_speed, double right_speed) {
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}

int main(int argc, char **argv) {
  wb_robot_init();

  // Inisialisasi sensor
  WbDeviceTag sensors[4];
  sensors[0] = wb_robot_get_device("ground front left infrared sensor");  // Sensor depan kiri
  sensors[1] = wb_robot_get_device("ground front right infrared sensor"); // Sensor depan kanan
  sensors[2] = wb_robot_get_device("ground left infrared sensor");        // Sensor kiri luar
  sensors[3] = wb_robot_get_device("ground right infrared sensor");       // Sensor kanan luar

  for (int i = 0; i < 4; i++) {
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  // Inisialisasi motor
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  double last_error = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Membaca nilai sensor
    double sensor_values[4];
    for (int i = 0; i < 4; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    // Menentukan error
    double error = sensor_values[0] - sensor_values[1];

    // Menghitung sinyal kontrol PID
    double pid = KP * error + KD * (error - last_error);
    last_error = error;

    // Menampilkan nilai error untuk debugging
    printf("Error: %.2f\n", error);

    // Mengatur kecepatan motor berdasarkan sinyal kontrol PID
    double left_speed = MAX_SPEED + pid;
    double right_speed = MAX_SPEED - pid;

    // Memastikan kecepatan tidak melebihi MAX_SPEED
    if (left_speed > MAX_SPEED) {
      left_speed = MAX_SPEED;
    }
    if (right_speed > MAX_SPEED) {
      right_speed = MAX_SPEED;
    }

    // Mengatur kecepatan motor
    set_motor_speeds(left_motor, right_motor, left_speed, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
