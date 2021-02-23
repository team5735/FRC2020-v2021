/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;

public class TurnToAngleV2Command extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Drivetrain drivetrain;
	private final double gyroSetpoint;
	
	private final double kSTEER_SPEED = 0.2;

	private double inDeadbandTime = -1;
	private double degreeError = 0;
	
	public TurnToAngleV2Command(Drivetrain drivetrain, double gyroSetpoint) {
		this.drivetrain = drivetrain;
		this.gyroSetpoint = gyroSetpoint;
		
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("TURN TO ANGLE");
		inDeadbandTime = -1;
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double currentGyroAngle = drivetrain.getGyroAngle();
		
		double angleDifference = Pathfinder.boundHalfDegrees(gyroSetpoint - currentGyroAngle);
		angleDifference = angleDifference % 360.0;
		if (Math.abs(angleDifference) > 180.0) {
			angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
		}
		
		double direction = Math.signum(angleDifference); // left = positive, right = negative

		drivetrain.drive(new DriveSignal(ControlMode.PercentOutput, direction * -kSTEER_SPEED, direction * kSTEER_SPEED, 0));
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("TURN TARGET COMMAND | END");
		drivetrain.drive(DriveSignal.STOP);
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.deadband(degreeError, RobotConstants.TURN_TO_ANGLE_DEADBAND) == 0;
	}
}
	