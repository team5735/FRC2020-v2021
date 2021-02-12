/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.trajectory.jaci.CFEncoderFollower;
import frc.lib.trajectory.jaci.EncoderFollower;
import frc.lib.trajectory.jaci.ReverseEncoderFollower;
import frc.lib.util.DriveSignal;
import frc.lib.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
* An example command that uses an example subsystem.
*/
public class DriveFollowTrajectory extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Drivetrain s_drivetrain;
	private Trajectory leftTraj, rightTraj;
	private final boolean inverted;

	private double initial_gyro_angle;

	private CFEncoderFollower left, right;
	private boolean isTrajDone = false;
	
	/**
	* Creates a new FollowTrajectory.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public DriveFollowTrajectory(Drivetrain subsystem, Trajectory leftTraj, Trajectory rightTraj, boolean inverted) {
		s_drivetrain = subsystem;
		this.leftTraj = leftTraj;
		this.rightTraj = rightTraj;
		this.inverted = inverted;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements((Subsystem) subsystem);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("========== INITIALIZING ==========");
		
		initial_gyro_angle = s_drivetrain.getGyroAngle();

		if(inverted) {
			left = new ReverseEncoderFollower(leftTraj);
			right = new ReverseEncoderFollower(rightTraj);
		} else {
			left = new EncoderFollower(leftTraj);
			right = new EncoderFollower(rightTraj);
		}

		left.configureEncoder(s_drivetrain.getLeftSidePosition(), (int) RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV, RobotConstants.DT_WHEEL_DIAMETER);
		// left.configurePIDVA(0.0, 0, 0, 1 / RobotConstants.MAX_VELOCITY_DT, 0);
		left.configurePIDVA(0.0, 0.0, 0.0, 1, 0);

		right.configureEncoder(s_drivetrain.getRightSidePosition(), (int) RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV, RobotConstants.DT_WHEEL_DIAMETER);
		// right.configurePIDVA(0.0, 0, 0, 1 / RobotConstants.MAX_VELOCITY_DT, 0);
		right.configurePIDVA(0.0, 0.0, 0.0, 1, 0);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(isTrajDone) return;
		int leftPos = s_drivetrain.getLeftSidePosition();
		int rightPos = s_drivetrain.getRightSidePosition();
		System.out.println("Left E: " + leftPos + ", Right E: " + rightPos);
		// RETURNS in PERCENT OUTPUT
		double l = left.calculate((inverted ? -1 : 1) * leftPos);
		double r = right.calculate((inverted ? -1 : 1) * rightPos);
		
		double gyro_heading = s_drivetrain.getGyroAngle() - initial_gyro_angle;
		double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
		SmartDashboard.putNumber("Gyro Angle", gyro_heading);
		// System.out.println("Heading: " + gyro_heading + " | Wanted: " + desired_heading);
		
		// This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		angleDifference = angleDifference % 360.0;
		if (Math.abs(angleDifference) > 180.0) {
			angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
		} 
		// if(angleDifference > 185) angleDifference = 0;
		
		double turn = RobotConstants.kTURN_CORRECTION * angleDifference;// * RobotConstants.MAX_VELOCITY_DT_TICKS;
		// double turn = 0;
		
		System.out.println("@@@@@@@@@@@@@@@ Left: " + (l + turn) + ", Right: " + (r - turn) + ", Angle Diff: " + angleDifference + ", Turn: " + turn);
		
		l = Units.dtRpmToTicks(Units.dtMetersPerSecondToRpm(l));
		r = Units.dtRpmToTicks(Units.dtMetersPerSecondToRpm(r));

		// s_drivetrain.drive(new DriveSignal(ControlMode.Velocity, l + turn, r - turn, 0));
		if(inverted) {
			s_drivetrain.drive(new DriveSignal(ControlMode.Velocity, -(l + turn), -(r - turn), 0));
		} else {
			s_drivetrain.drive(new DriveSignal(ControlMode.Velocity, l + turn, r - turn, 0));
		}	
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("DONE: L current:" + Units.dtRotationsToMeters(Units.dtTickstoRotations(s_drivetrain.getLeftSidePosition())) / 6.2222 + ", R current: " + Units.dtRotationsToMeters(Units.dtTickstoRotations(s_drivetrain.getRightSidePosition())) / 6.2222);
		s_drivetrain.drive(DriveSignal.NEUTRAL);
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(left.isFinished() && right.isFinished()) {
			isTrajDone = true;
			return true;
		}
		return false;
	}
}
