#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::MotorGroup left_motors({1, 4}, pros::MotorGearset::green);
pros::MotorGroup right_motors({-2, -3}, pros::MotorGearset::green);

// intake, roller, and top hat
pros::Motor intake(7, pros::MotorGearset::blue);
pros::Motor intake_roller(8, pros::MotorGearset::blue);
pros::Motor top_hat(9, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9, // 9 inch track width
                              lemlib::Omniwheel::NEW_325, // 3.25" wheels
                              333.33, // drivetrain rpm
                              2 // horizontal drift (omni wheels)
);

// imu
pros::Imu imu(10);

// distance sensors
pros::Distance distance_front(14); // forward-facing (Y axis)
pros::Distance distance_left(15);  // left-facing (X axis)

// odom sensors (using IMEs only, no tracking wheels)
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // kP
                                              0, // kI
                                              3, // kD
                                              3, // anti windup
                                              1, // small error range (inches)
                                              100, // small error timeout (ms)
                                              3, // large error range (inches)
                                              500, // large error timeout (ms)
                                              20 // slew
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // kP
                                              0, // kI
                                              10, // kD
                                              3, // anti windup
                                              1, // small error range (degrees)
                                              100, // small error timeout (ms)
                                              3, // large error range (degrees)
                                              500, // large error timeout (ms)
                                              0 // slew (disabled)
);

// drive curves for input scaling
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);


void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // screen task for odom and distance sensor display
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Front dist: %d mm", distance_front.get());
            pros::lcd::print(4, "Left dist: %d mm", distance_left.get());
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

// Align robot to be ~2cm (20mm) from the wall using distance sensors
// Uses left distance sensor for side wall and front for forward wall
void alignToWall(double target_mm = 20.0, int timeout_ms = 1500) {
    int start = pros::millis();
    while (pros::millis() - start < timeout_ms) {
        double dist = distance_left.get();
        if (dist == 0) break; // sensor error, bail
        double error = dist - target_mm;
        if (fabs(error) < 5) break; // within 5mm tolerance

        // P-controller: small differential correction to push toward/away from wall
        double correction = error * 0.4;
        if (correction > 25) correction = 25;
        if (correction < -25) correction = -25;

        left_motors.move(correction);
        right_motors.move(-correction);
        pros::delay(10);
    }
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(50);
}

void autonomous() {
    // Set initial pose (converted from path.jerryio cm to inches)
    // Start position: (167.596, -37.374) cm = (65.98, -14.71) in
    chassis.setPose(65.98, -14.71, 0);

    // ===== FIRST SETUP: Push forward along wall, passing park zone =====
    // Spin intake + roller (no high top) between first 2 coords
    intake.move_velocity(100);
    intake_roller.move_velocity(100);

    // Move forward to (168.84, 38.10) cm = (66.47, 15.00) in
    chassis.moveToPoint(66.47, 15.00, 3000, {.forwards=false});
    // Stop intake + roller after reaching first waypoint
    intake.move_velocity(0);
    intake_roller.move_velocity(0);

    // Align 2cm from wall to ensure we pass the park zone completely
    alignToWall(20.0, 1500);

    // ===== Curve left to push game elements =====
    // (59.77, 61.74) cm = (23.53, 24.31) in
    chassis.moveToPoint(23.53, 24.31, 3000);

    // (14.56, 14.47) cm = (5.73, 5.69) in
    chassis.moveToPoint(5.73, 5.69, 3000);

    // ===== Push back right =====
    // (118.66, 60.08) cm = (46.72, 23.66) in
    chassis.moveToPoint(46.72, 23.66, 4000);

    // ===== Move up to upper zone =====
    // (117.42, 118.97) cm = (46.23, 46.84) in
    chassis.moveToPoint(46.23, 46.84, 3000);

    // (174.23, 118.97) cm = (68.60, 46.84) in
    chassis.moveToPoint(68.60, 46.84, 3000);

    // ===== Curve to sweep upper field =====
    // (117.00, 155.88) cm = (46.06, 61.37) in
    chassis.moveToPoint(46.06, 61.37, 3000);

    // ===== Long sweep across top to left side =====
    // (-120.22, 160.86) cm = (-47.33, 63.33) in
    chassis.moveToPoint(-47.33, 63.33, 5000);

    // ===== Drop down on left side =====
    // (-121.05, 119.80) cm = (-47.66, 47.17) in
    chassis.moveToPoint(-47.66, 47.17, 3000);

    // ===== Sweep right then back left =====
    // (-57.18, 120.22) cm = (-22.51, 47.33) in
    chassis.moveToPoint(-22.51, 47.33, 3000);

    // (-171.23, 119.39) cm = (-67.41, 46.97) in
    chassis.moveToPoint(-67.41, 46.97, 4000);

    // ===== Drop down left side =====
    // (-118.56, 76.67) cm = (-46.68, 30.19) in
    chassis.moveToPoint(-46.68, 30.19, 3000);

    // (-162.93, 60.08) cm = (-64.15, 23.66) in
    chassis.moveToPoint(-64.15, 23.66, 3000);

    // ===== Move down to lower half =====
    // (-172.47, -56.87) cm = (-67.90, -22.39) in
    chassis.moveToPoint(-67.90, -22.39, 4000);

    // (-122.71, -58.53) cm = (-48.31, -23.04) in
    chassis.moveToPoint(-48.31, -23.04, 3000);

    // (-125.61, -118.66) cm = (-49.45, -46.72) in
    chassis.moveToPoint(-49.45, -46.72, 3000);

    // (-168.74, -119.49) cm = (-66.43, -47.04) in
    chassis.moveToPoint(-66.43, -47.04, 3000);

    // ===== Sweep across bottom =====
    // (-118.15, -160.55) cm = (-46.51, -63.21) in
    chassis.moveToPoint(-46.51, -63.21, 3000);

    // ===== Long sweep across bottom to right side =====
    // (121.15, -158.89) cm = (47.70, -62.56) in
    chassis.moveToPoint(47.70, -62.56, 5000);

    // ===== Work right side lower =====
    // (122.39, -119.90) cm = (48.19, -47.21) in
    chassis.moveToPoint(48.19, -47.21, 3000);

    // (50.65, -121.56) cm = (19.94, -47.86) in
    chassis.moveToPoint(19.94, -47.86, 3000);

    // (172.57, -118.66) cm = (67.94, -46.72) in
    chassis.moveToPoint(67.94, -46.72, 4000);

    // ===== DIRECTION CHANGE: Score on lower stake =====
    // No wall alignment from here - heading to score
    // (149.76, -110.78) cm = (58.96, -43.61) in
    chassis.moveToPoint(58.96, -43.61, 2000);

    // Final approach to lower stake
    // (148.52, 5.34) cm = (58.47, 2.10) in
    chassis.moveToPoint(58.47, 2.10, 4000);
}

void opcontrol() {
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(25);
    }
}
