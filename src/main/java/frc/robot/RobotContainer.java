/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.lib.controllers.BobXboxController;
import frc.lib.util.DriveSignal;
import frc.robot.commandgroups.SixBallAutoCommand;
import frc.robot.commandgroups.TurnAndShootCommand;
import frc.robot.commandgroups.TurnAndShootCommandSemiAuto;
import frc.robot.commands.climber.ElevatorMoveCommand;
import frc.robot.commands.climber.WinchMoveCommand;
import frc.robot.commands.drivetrain.ChangeDriveMode;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.commands.intake.ZeroIntakeCommand;
import frc.robot.commands.shooter.MoveBananaCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.ReverseShooterCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.Trajectories;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Winch;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public final ColorMatcher colorMatcher = new ColorMatcher();
	public final ColorSpinner colorSpinner = new ColorSpinner();
	public final Drivetrain drivetrain = new Drivetrain();
	public final Shooter shooter = new Shooter();
	public final Banana banana = new Banana();
	public final Feeder feeder = new Feeder();
	public final Conveyor conveyor = new Conveyor();
	public final IntakeArm intakeArm = new IntakeArm();
	public final Telescope telescope = new Telescope();
	public final TrajectoryGenerator trajectoryGenerator = new TrajectoryGenerator();
	public final Vision vision = new Vision();
	public final Winch winch = new Winch();
	
	public static final BobXboxController driverController = new BobXboxController(0);
	public static final BobXboxController subsystemController = new BobXboxController(1);
	
	/**
	* The container for the robot.  Contains subsystems, OI devices, and commands.
	*/
	public RobotContainer() {
		// Configure the button bindings
		configureDriverBindings();
		configureSubsystemBindings();
		SmartDashboard.putNumber("RPM", RobotConstants.FLYWHEEL_PRESET_LINE);

		getRamseteCommand();
		
		// TrajectoryGenerator.exportTrajectories(sixBall, "MiddleToTrench");
	}
	
	public void stopAll() {
		CommandScheduler.getInstance().cancelAll();
		shooter.slowDown();
		intakeArm.intakeBall(0, false);
		vision.disableTracking();
		drivetrain.drive(DriveSignal.NEUTRAL);
	}
	
	private void configureDriverBindings() {
		// subsystemController.xButton.whenPressed(new ColorSpinCommand(colorSpinner, 4));
		// subsystemController.bButton.whenPressed(new ColorMatchCommand(colorSpinner, colorMatcher));
		// driverController.aButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_DEPLOYED));
		// driverController.bButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_RETRACTED));
		// driverController.yButton.whenPressed(new ResetGyroAngle(drivetrain));
		// driverController.xButton.whenPressed(new ChangeDriveMode(drivetrain));
		
		driverController.rightBumper.whileActiveContinuous(new AngleIntakeCommand(intakeArm, false));
		driverController.leftBumper.whileActiveContinuous(new AngleIntakeCommand(intakeArm, true));
		
		driverController.rightTriggerButton.whileActiveContinuous(new IntakeBallCommand(intakeArm));
		driverController.leftTriggerButton.whileActiveContinuous(new IntakeBallCommand(intakeArm));
		
		// driverController.Dpad.Right.whileHeld(new IntakeBallCommand(intake, 0.2, false));
		// driverController.Dpad.Left.whileHeld(new IntakeBallCommand(intake, 0.2, true));
		// driverController.Dpad.Left.whenPressed(new RampShooterCommand(shooter, banana, RobotConstants.FLYWHEEL_PRESET_TRENCH));
		// driverController.Dpad.Right.whileHeld();
		
		
		// driverController.startButton.whenPressed(new ZeroIntakeCommand(intake));
		//   trajectoryGenerator.getTrajectorySet().sideStartToNearScale.left, (DrivetrainTrajectory)drivetrain));
	}
	
	private void configureSubsystemBindings() {
		// subsystemController.aButton.whenPressed(new RampShooterCommand(shooter, 3750));
		// subsystemController.aButton.whenReleased(new RampShooterCommand(shooter, 0));
		
		subsystemController.Dpad.Up.whenPressed(new RampShooterCommand(shooter, vision, banana, feeder, 3000.00));
		subsystemController.Dpad.Down.whenPressed(new RampShooterCommand(shooter, vision, banana, feeder, 0.0));
		
		subsystemController.aButton.whenPressed(new InstantCommand(vision::getDistanceToTarget, vision));
		subsystemController.bButton.whenPressed(new InstantCommand(vision::disableTracking, vision));
		
		// subsystemController.aButton.whenPressed(new TurnAndShootCommandSemiAuto(vision, drivetrain, feeder, conveyor, intakeArm , shooter, banana));
		// subsystemController.aButton.whenPressed(new MoveBananaCommand(banana, 1000));
		// subsystemController.bButton.whenPressed(new MoveBananaCommand(banana, 2500));
		// subsystemController.bButton.whileHeld(new WinchMoveCommand(winch));
		// subsystemController.yButton.whenPressed(new MoveBananaCommand(banana, 0));
		// subsystemController.yButton.whenPressed();
		
		// subsystemController.Dpad.Up.whileHeld(new ElevatorMoveCommand(telescope, false));
		// subsystemController.Dpad.Down.whileHeld(new ElevatorMoveCommand(telescope, true));
		
		// subsystemController.Dpad.Up.whenPressed(new RampShooterCommand(shooter, banana, 3650));
		// subsystemController.Dpad.Right.whenPressed(new RampShooterCommand(shooter, banana, RobotConstants.FLYWHEEL_PRESET_TRENCH));
		// subsystemController.Dpad.Left.whenPressed(new RampShooterCommand(shooter, banana, RobotConstants.FLYWHEEL_PRESET_BEHINDCOLORWHEEL));
		
		subsystemController.rightBumper.whileActiveContinuous(new MoveConveyorCommand(conveyor, feeder, shooter, 0.420, false));
		subsystemController.leftBumper.whileActiveContinuous(new MoveConveyorCommand(conveyor, feeder, shooter, 0.420, true));
		
		subsystemController.rightTriggerButton.whileActiveContinuous(new FeedShooterCommand(feeder, shooter, false));
		subsystemController.leftTriggerButton.whileActiveContinuous(new FeedShooterCommand(feeder, shooter, true));
		
		subsystemController.startButton.whileActiveContinuous(new ReverseShooterCommand(shooter));
		
	}
	
	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		return getRamseteCommand();
		// return new DriveFollowTrajectory(drivetrain, Trajectories.SnakeCurve[0], Trajectories.SnakeCurve[1], false);
		// return new DriveFollowTrajectory(drivetrain, Trajectories.MoveOneMeter[0], Trajectories.MoveOneMeter[1], false);
		// return new SixBallAutoCommand(vision, drivetrain, feeder, conveyor, intakeArm, shooter, banana);
	}
	
	public Command getRamseteCommand() {

		var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(RobotConstants.ksVolts,
			RobotConstants.kvVoltSecondsPerMeter,
			RobotConstants.kaVoltSecondsSquaredPerMeter),
            RobotConstants.kDriveKinematics,
            10);

		// Create config for trajectory
		TrajectoryConfig config =
			new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
			RobotConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(RobotConstants.kDriveKinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);

		// An example trajectory to follow.  All units in meters.
		Trajectory trajectory = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(
				new Translation2d(1, 0)
				// new Translation2d(2, -1)
			),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(1.524, 0, new Rotation2d(0)),
			// Pass config
			config
		);
		// Trajectory trajectory = Trajectories.FiveFeet;
		
		RamseteCommand ramseteCommand = new RamseteCommand(
		trajectory,
		drivetrain::getPose,
		new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
		new SimpleMotorFeedforward(RobotConstants.ksVolts,
		RobotConstants.kvVoltSecondsPerMeter,
		RobotConstants.kaVoltSecondsSquaredPerMeter),
		RobotConstants.kDriveKinematics,
		drivetrain::getWheelSpeeds,
		new PIDController(RobotConstants.kPDriveVel, 0, 0),
		new PIDController(RobotConstants.kPDriveVel, 0, 0),
		// RamseteCommand passes volts to the callback
		drivetrain::tankDriveVolts,
		drivetrain
		);
		
		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose());
		
		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
	}
	
}
