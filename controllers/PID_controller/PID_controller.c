#include <webots/robot.h>
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define MAX_MOTOR_SPEED 47.61  // Kecepatan maksimum motor

// Konstanta PID
#define KP 1.7
#define KI 0
#define KD 0.3

// Target nilai sensor tengah
#define TARGET_SENSOR_VALUE 400

// Fungsi untuk mendapatkan nilai error
double get_error(double center, double left, double right) {
    double target = TARGET_SENSOR_VALUE;
    double current = center;
    return current - target;
}

int main() {
    wb_robot_init();

    // Inisialisasi motor
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    // Inisialisasi sensor
    WbDeviceTag ds_center = wb_robot_get_device("ds_center");
    WbDeviceTag ds_left = wb_robot_get_device("ds_left");
    WbDeviceTag ds_right = wb_robot_get_device("ds_right");
    wb_distance_sensor_enable(ds_center, TIME_STEP);
    wb_distance_sensor_enable(ds_left, TIME_STEP);
    wb_distance_sensor_enable(ds_right, TIME_STEP);

    // Variabel PID
    double prev_error = 0;
    double integral = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        // Baca nilai dari sensor
        double center_value = wb_distance_sensor_get_value(ds_center);
        double left_value = wb_distance_sensor_get_value(ds_left);
        double right_value = wb_distance_sensor_get_value(ds_right);

        // Mencetak nilai sensor
        printf("Center: %.2f, Left: %.2f, Right: %.2f\n", center_value, left_value, right_value);

        // Hitung nilai error
        double error = get_error(center_value, left_value, right_value);

        // Hitung nilai PID
        double proportional = KP * error;
        integral += error;
        double derivative = error - prev_error;
        double pid_value = proportional + KI * integral + KD * derivative;

        // Simpan nilai error sebelumnya
        prev_error = error;

        // Mencetak output PID
        printf("PID Output: %.2f\n", pid_value);

        // Kendalikan motor dengan nilai PID
        double left_speed = MAX_SPEED - pid_value;
        double right_speed = MAX_SPEED + pid_value;

        // Batasi kecepatan motor agar tidak melebihi batas maksimum
        if (left_speed > MAX_MOTOR_SPEED) {
            left_speed = MAX_MOTOR_SPEED;
        } else if (left_speed < -MAX_MOTOR_SPEED) {
            left_speed = -MAX_MOTOR_SPEED;
        }

        if (right_speed > MAX_MOTOR_SPEED) {
            right_speed = MAX_MOTOR_SPEED;
        } else if (right_speed < -MAX_MOTOR_SPEED) {
            right_speed = -MAX_MOTOR_SPEED;
        }

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }

    wb_robot_cleanup();

    return 0;
}
