/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.BobXboxController;
import frc.lib.util.DriveSignal;
import frc.robot.commandgroups.SixBallAutoCommand;
import frc.robot.commandgroups.TurnAndShootCommand;
import frc.robot.commandgroups.TurnAndShootCommandSemiAuto;
import frc.robot.commands.climber.ElevatorMoveCommand;
import frc.robot.commands.climber.WinchMoveCommand;
import frc.robot.commands.drivetrain.ChangeDriveMode;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.drivetrain.ResetGyroAngle;
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
import frc.robot.subsystems.Drivetrain.DriveMode;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;


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

		// TrajectoryGenerator.exportTrajectories(sixBall, "MiddleToTrench");
	}

	public void stopAll() {
		CommandScheduler.getInstance().cancelAll();
		shooter.slowDown();
		intakeArm.intakeBall(0, false);
		vision.disableTracking();
		drivetrain.drive(DriveSignal.NEUTRAL);
		drivetrain.setDriveMode(DriveMode.STATIC_DRIVE);
	}
	
	private void configureDriverBindings() {
		// subsystemController.xButton.whenPressed(new ColorSpinCommand(colorSpinner, 4));
		// subsystemController.bButton.whenPressed(new ColorMatchCommand(colorSpinner, colorMatcher));
		// driverController.aButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_DEPLOYED));
		// driverController.bButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_RETRACTED));
		driverController.yButton.whenPressed(new ResetGyroAngle(drivetrain));
		driverController.xButton.whenPressed(new ChangeDriveMode(drivetrain));

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
		return new DriveFollowTrajectory(drivetrain, Trajectories.CurveRight1M[0], Trajectories.CurveRight1M[1], false);
		// return new DriveFollowTrajectory(drivetrain, Trajectories.MoveOneMeter[0], Trajectories.MoveOneMeter[1], false);
		// return new SixBallAutoCommand(vision, drivetrain, feeder, conveyor, intakeArm, shooter, banana);
	}
	
}
