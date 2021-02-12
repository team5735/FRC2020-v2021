/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

/**
 * An example command that uses an example subsystem.
 */
public class AngleIntakeCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeArm intakeArm;
	private final double position;
	private final boolean inverted; // inverted = up

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public AngleIntakeCommand(IntakeArm intakeArm, double position) {
		this.intakeArm = intakeArm;
		this.position = position;
		this.inverted = false;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intakeArm);
	}

	public AngleIntakeCommand(IntakeArm intakeArm, boolean inverted) {
		this.intakeArm = intakeArm;
		this.position = -1;
		this.inverted = inverted;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// intake.moveArm(ControlMode.Position, position);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// System.out.println(intake.getPosition());
		// double position = intake.getPosition();
		// if(position < RobotConstants.INTAKE_POSITION_RETRACTED ||
		// position > RobotConstants.INTAKE_POSITION_DEPLOYED) end(true);
		intakeArm.moveArm(ControlMode.PercentOutput, (inverted ? -1 : 1) * 0.26);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// intake.moveArm(ControlMode.Position, intake.getPosition()); // stay
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return Util.deadband(intake.getPosition(),
		// RobotConstants.INTAKE_POSITION_DEADBAND) == 0;
		return false;
	}
}
