/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Conveyor extends SubsystemBase {

	private final VictorSPX conveyorRoller;

	// private final DigitalInput retractedLimitSwitch, deployedLimitSwitch;

	/**
	 * Creates a new Intake.
	 */
	public Conveyor() {
		conveyorRoller = new VictorSPX(RobotConstants.CONVEYOR_ID);
		conveyorRoller.configFactoryDefault();
		conveyorRoller.setInverted(true);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void rollConveyor(double speed, boolean inverted) {
		conveyorRoller.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * speed);
	}
}
