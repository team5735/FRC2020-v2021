/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;


public class Banana extends SubsystemBase {

	private final TalonSRX banana;
	// private final DigitalInput retractedLimitSwitch;

	/**
	 * Creates a new Banana.
	 */
	public Banana() {
		banana = new TalonSRX(RobotConstants.BANANA_ID);
		banana.configFactoryDefault();
		banana.config_kP(0, 1.1435);
		banana.config_kI(0, 0.0009);
		banana.config_kD(0, 0);

		banana.setSelectedSensorPosition(0);

		// retractedLimitSwitch = new DigitalInput(9);
	}

	public double getPosition() {
		return banana.getSelectedSensorPosition();
	}

	public void moveBanana(ControlMode mode, double value) {
		if(isRetractedLimitHit() && value < 0) return;
		// if(getPosition() + value > RobotConstants.BANANA_POSITION_DEPLOYED) return;
		banana.set(mode, value);
	}

	public boolean isRetractedLimitHit() {
		return false;
		// return !retractedLimitSwitch.get();
	}

	public void retract() {
		// banana.set(ControlMode.Position, RobotConstants.BANANA_POSITION_RETRACTED);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
