#include "main.h"
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(3);
	pros::Motor lift_mtr(4);
	pros::Motor roller(6);


	//variables for PID
  static const double liftKp = 1.5;
	static double liftSetVal = 0;
	static double liftErr = 0;
	static double currLiftVal = 0;
	static bool liftHold = false;


	//roller
	static bool rollerHold = false;


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


		//base
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = -right;



		//lift
		currLiftVal = lift_mtr.get_position();
		liftErr = liftSetVal - currLiftVal;
		//lift_mtr = liftErr * liftKp;
		pros::lcd::print(3, "liftHold: %d", liftHold);
		pros::lcd::print(4, "currLiftVal: %f", currLiftVal);
		pros::lcd::print(5, "liftSetVal: %f", liftSetVal);
		pros::lcd::print(6, "liftErr: %f", liftErr);
		pros::lcd::print(7, "lift_mtr: %f", (liftErr * liftKp));

		//Manual lift code
		if (master.get_digital(DIGITAL_R1)) {
			liftHold = false;
			lift_mtr = -127;
		}
	  else if (master.get_digital(DIGITAL_R2)) {
			liftHold = false;
			lift_mtr = 127;
		}
		else if (liftHold == false) {
			liftHold = true;
			liftSetVal = currLiftVal;
		}
		//lift PID hold
		if (liftHold) {
			lift_mtr = liftErr * liftKp;
		}



		//roller
		if (master.get_digital(DIGITAL_L1)) {
			rollerHold = false;
			roller = 50;
		}
		else if ((rollerHold == false) && (master.get_digital(DIGITAL_L2) == false)) {
			roller = 0;
		}
	  else if (master.get_digital(DIGITAL_L2)) {
			rollerHold = true;
			roller = -50;
		}

		//constant power hold
		/*
		if (rollerHold) {
			roller = 50;
		}
		*/


		pros::delay(20);
	}
}
