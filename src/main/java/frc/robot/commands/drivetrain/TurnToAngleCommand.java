/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToAngleCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Drivetrain drivetrain;
	private final double gyroSetpoint;

	private final PIDController turnPID;
	
	private double inDeadbandTime = -1;
	private double degreeError = 0;
	
	public TurnToAngleCommand(Drivetrain drivetrain, double gyroSetpoint) {
		this.drivetrain = drivetrain;
		this.gyroSetpoint = gyroSetpoint;

		this.turnPID = new PIDController(RobotConstants.VISION_STEER_kP, RobotConstants.VISION_STEER_kI, RobotConstants.VISION_STEER_kD);
		turnPID.setTolerance(RobotConstants.TURN_TO_ANGLE_DEADBAND);
		//pid.setIntegratorRange(-0.5, 0.5); use this if it "winds up" and overshoots
		
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
		degreeError = gyroSetpoint - drivetrain.getHeading();
		double steer_cmd = Util.limit(turnPID.calculate(degreeError, 0), -1, 1); 
		
		drivetrain.drive(new DriveSignal(ControlMode.PercentOutput, -steer_cmd, steer_cmd, 0));
		if(Util.deadband(degreeError, RobotConstants.VISION_TARGET_DEADBAND) == 0) {
			if(inDeadbandTime < 0) {
				inDeadbandTime = Timer.getFPGATimestamp();	
			}
		} else {
			inDeadbandTime = -1;
		}
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("TURN TARGET COMMAND | END");
		drivetrain.drive(DriveSignal.NEUTRAL);
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		SmartDashboard.putNumber("InDeadbandTime", inDeadbandTime);
		// return false;
		//		if greater than 0 and	80 milliseconds have passed		and		we are at setpoint
		return (inDeadbandTime > 0) && (inDeadbandTime + 0.08 < Timer.getFPGATimestamp()) && Util.deadband(degreeError, RobotConstants.TURN_TO_ANGLE_DEADBAND) == 0;
	}

	// public boolean getAsBoolean() {
		// return (inDeadbandTime > 0) && (inDeadbandTime + 0.08 < Timer.getFPGATimestamp()) && Util.deadband(vision.getTX(), RobotConstants.VISION_TARGET_DEADBAND) == 0;
	// }
}
