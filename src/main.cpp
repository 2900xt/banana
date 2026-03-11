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
    // Start: (167.596, -37.374) cm = (65.98, -14.71) in
    chassis.setPose(65.98, -14.71, 0);

    // ===== FIRST SETUP: Push forward along wall, passing park zone =====
    // Spin intake + roller (no high top) between first 2 coords
    intake.move_velocity(100);
    intake_roller.move_velocity(100);

    // (168.84, 38.83) cm = (66.47, 15.29) in
    chassis.moveToPoint(66.47, 15.29, 3000, {.forwards=false});
    // Stop intake + roller after reaching first waypoint
    intake.move_velocity(0);
    intake_roller.move_velocity(0);

    // Align 2cm from wall to ensure we pass the park zone completely
    alignToWall(20.0, 1500);

    // (60.789, 61.515) cm = (23.93, 24.22) in
    chassis.moveToPoint(23.93, 24.22, 3000);

    // (14.572, 14.473) cm = (5.74, 5.70) in
    chassis.moveToPoint(5.74, 5.70, 3000);

    // (118.628, 60.071) cm = (46.70, 23.65) in
    chassis.moveToPoint(46.70, 23.65, 4000);

    // (117.406, 119.887) cm = (46.22, 47.20) in
    chassis.moveToPoint(46.22, 47.20, 3000);

    // (174.814, 118.978) cm = (68.82, 46.84) in
    chassis.moveToPoint(68.82, 46.84, 3000);

    // (117.018, 155.873) cm = (46.07, 61.37) in
    chassis.moveToPoint(46.07, 61.37, 3000);

    // (-117.552, 160.809) cm = (-46.28, 63.31) in
    chassis.moveToPoint(-46.28, 63.31, 5000);

    // (-121.048, 119.799) cm = (-47.66, 47.17) in
    chassis.moveToPoint(-47.66, 47.17, 3000);

    // (-57.796, 120.211) cm = (-22.76, 47.33) in
    chassis.moveToPoint(-22.76, 47.33, 3000);

    // (-171.112, 119.39) cm = (-67.37, 47.00) in
    chassis.moveToPoint(-67.37, 47.00, 4000);

    // (-118.169, 76.352) cm = (-46.52, 30.06) in
    chassis.moveToPoint(-46.52, 30.06, 3000);

    // (-163.278, 59.947) cm = (-64.28, 23.60) in
    chassis.moveToPoint(-64.28, 23.60, 3000);

    // (-172.399, -55.917) cm = (-67.87, -22.01) in
    chassis.moveToPoint(-67.87, -22.01, 4000);

    // (-122.187, -58.549) cm = (-48.11, -23.05) in
    chassis.moveToPoint(-48.11, -23.05, 3000);

    // (-125.576, -118.05) cm = (-49.44, -46.48) in
    chassis.moveToPoint(-49.44, -46.48, 3000);

    // (-168.696, -119.488) cm = (-66.42, -47.04) in
    chassis.moveToPoint(-66.42, -47.04, 3000);

    // (-117.485, -161.057) cm = (-46.25, -63.41) in
    chassis.moveToPoint(-46.25, -63.41, 3000);

    // (118.738, -158.908) cm = (46.75, -62.56) in
    chassis.moveToPoint(46.75, -62.56, 5000);

    // (122.366, -118.504) cm = (48.17, -46.66) in
    chassis.moveToPoint(48.17, -46.66, 3000);

    // (51.138, -121.548) cm = (20.13, -47.85) in
    chassis.moveToPoint(20.13, -47.85, 3000);

    // (172.606, -118.658) cm = (67.95, -46.72) in
    chassis.moveToPoint(67.95, -46.72, 4000);

    // ===== DIRECTION CHANGE: Score on lower stake =====
    // (149.524, -110.684) cm = (58.87, -43.58) in
    chassis.moveToPoint(58.87, -43.58, 2000);

    // Final approach to lower stake
    // (148.519, 5.342) cm = (58.47, 2.10) in
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
