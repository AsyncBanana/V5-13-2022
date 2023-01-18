#include "main.h"
#include "okapi/api.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/impl/device/button/adiButton.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
using namespace okapi;

Motor flywheel_launch(9);
Motor flywheel_spinner(10);
Motor intake(16);
Motor indexer(5);
Motor spinner(4);
std::shared_ptr<OdomChassisController> drivetrain =
    ChassisControllerBuilder()
        .withMotors({1, 2, 3}, {6, 7, 8})
        .withGains({0.001, 0, 0.0001}, // Distance controller gains
                   {0.001, 0, 0.0001}, // Turn controller gains
                   {0.001, 0, 0.0001})
        .withDimensions(AbstractMotor::gearset::green,
                        {{4_in, 11.5_in}, imev5GreenTPR})
        .withOdometry()
        .buildOdometry();
Controller controller;
ControllerButton controller_l1(ControllerDigital::L1);
ControllerButton controller_l2(ControllerDigital::L2);
ControllerButton controller_a(ControllerDigital::A);
ControllerButton controller_x(ControllerDigital::X);
enum positions { p_spinner = 1, p_other = 2, p_full = 3 };
ControllerButton controller_y(ControllerDigital::Y);
enum teams { t_red = 1, t_blue = 2 };
int g_position;
int g_team;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { pros::lcd::initialize(); }
void init_auto() {
  g_team = 0;
  g_position = 0;
  controller.clear();
  controller.setText(0, 0, "A:red Y:blue");
  while (g_team == 0) {
    if (controller_a.changedToPressed()) {
      g_team = t_red;
    } else if (controller_y.changedToPressed()) {
      g_team = t_blue;
    }
  }
  controller.clear();
  controller.setText(0, 0, "A:roll Y:other X:all");
  while (g_position == 0) {
    if (controller_a.changedToPressed()) {
      g_position = p_spinner;
    } else if (controller_y.changedToPressed()) {
      g_position = p_other;
    } else if (controller_x.changedToPressed()) {
      g_position = p_full;
    }
  }
}
/**
 * Runs while the robot is in the disabled state of Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol.
 * When the robot is enabled, this task will exit.
 */
void disabled() {}
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  init_auto();
  controller.clear();
}

/**
 * Turns to the launcher and launches a disk. Automatically adapt to teams
 */
void launch() {
  if (g_team == t_red) {
    // Red shooter = 4.5, 4.5
    drivetrain->turnToPoint({4.5_ft, 4.5_ft});
  } else {
    // Blue shooter = -4.5, -4.5
    drivetrain->turnToPoint({-4.5_ft, -4.5_ft});
  }
  // LAUNCH
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  controller.clear();
  controller.setText(1, 0, "Starting Autonomous");
  if (g_position == p_spinner || g_position == p_full) {
    drivetrain->setState({-4_ft, 3_ft, 90_deg});
  } else {
    drivetrain->setState({-6_ft, 1_ft, 0_deg});
  }
  if (g_team == t_blue) {
    OdomState state = drivetrain->getState();
    drivetrain->setState(
        {-state.x, -state.y, state.theta + okapi::degree * 180});
  }
  // Shared spinner code
  if (g_position == p_spinner || g_position == p_full) {
    drivetrain->driveToPoint({-5_ft, 3_ft}, true);
    spinner.moveVelocity(100);
    pros::delay(2000);
    spinner.moveVelocity(0);
  }
  // Full Auto Code
  if (g_position == p_full) {
    // Fire preloads
    launch();
    // Get first part of side line
    // drivetrain->driveToPoint({-4_ft,2_ft}); // Might not be necessary
    drivetrain->driveToPoint({-2_ft, 0_ft});
    launch();
    // Get second part of side line
    drivetrain->driveToPoint({2.5_ft, -4.5_ft});
    launch();
    drivetrain->driveToPoint({-2_ft, -5_ft});
    drivetrain->driveToPoint({-2_ft, -1_ft});
    launch();
    drivetrain->driveToPoint({-5_ft, -1.5_ft});
    launch();
    // Expand? :)
  }
  // Spinner Teamwork Auto - Move into launch position
  if (g_position == p_spinner) {
    // Fire preloads
    drivetrain->driveToPoint({-4_ft, 3_ft});
  }
  // All Teamwork Auto - Launch preloads
  if (g_position == p_other || g_position == p_spinner) {
    launch();
  }
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  if (!pros::competition::is_connected()) {
    controller.setText(0, 0, "A to start autonomous");
  }
  while (true) {
    int SpinnerBackward = controller_l1.isPressed();
    int SpinnerForward = controller_l2.isPressed();
    int APressed = controller_a.changedToPressed();
    if (APressed) {
      init_auto();
      autonomous();
    }
    spinner.moveVoltage(SpinnerForward * 12000 - SpinnerBackward * 12000);
    drivetrain->getModel()->arcade(
        controller.getAnalog(ControllerAnalog::leftY),
        controller.getAnalog(ControllerAnalog::leftX));
    pros::delay(4);
  }
}
