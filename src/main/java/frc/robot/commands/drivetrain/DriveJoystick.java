/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.helper.HDriveHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class DriveJoystick extends CommandBase {
	
	private Drivetrain drivetrain;
	
	// private double maxV = 0;
	
	/**
	* Creates a new DriveJoystick.
	*/
	public DriveJoystick(Drivetrain drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		
		addRequirements((Subsystem) drivetrain);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if(drivetrain.getDriveMode() == DriveMode.DISABLED) return;

		double forward = Util.deadband(RobotContainer.driverController.rightStick.getY(), 0.16);
		double normal = Util.deadband(RobotContainer.driverController.rightStick.getX(), 0.2);
		double turn = Util.deadband(RobotContainer.driverController.leftStick.getX(), 0.16);

		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle());
		SmartDashboard.putNumber("Left Velocity", drivetrain.getLeftVelocity());
		SmartDashboard.putNumber("Right Velocity", drivetrain.getRightVelocity());

		/* 0.5x + 0.5x^3
		forward = (0.5 * forward) + (0.5 * Math.pow(forward, 3));
		normal = (0.5 * normal) + (0.5 * Math.pow(normal, 3));
		turn = (0.5 * turn) + (0.5 * Math.pow(turn, 3)); */

		/* Arctan
		forward = Math.atan((Math.PI / 2) * forward);
		normal = Math.atan((Math.PI / 2) * normal);
		turn = Math.atan((Math.PI / 2) * turn); */

		// x^2
		forward = Math.copySign(Math.pow(forward, 2), forward);
		normal = Math.copySign(Math.pow(normal, 2), normal);
		turn = Math.copySign(Math.pow(turn, 2), turn);

		/* x^3
		forward = Math.copySign(Math.pow(forward, 3), forward);
		normal = Math.copySign(Math.pow(normal, 3), normal);
		turn = Math.copySign(Math.pow(turn, 3), turn); */
		
		if(drivetrain.getDriveMode() == DriveMode.FIELD_CENTRIC) {
			drivetrain.drive(HDriveHelper.HdriveFieldCentric(forward, normal, turn, drivetrain.getGyroAngle()));
		} else {
			drivetrain.drive(HDriveHelper.HDrive(forward, normal, turn, false));
			// drivetrain.drive(new DriveSignal(ControlMode.Velocity, 0, 0, normal * RobotConstants.MAX_VELOCITY_NORMAL_TICKS));
		}
		
		// System.out.println("Gyro Angle: " + drivetrain.getGyroAngle());
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
