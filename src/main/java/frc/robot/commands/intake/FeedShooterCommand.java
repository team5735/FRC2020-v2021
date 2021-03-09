/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
* An example command that uses an example subsystem.
*/
public class FeedShooterCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Feeder feeder;
	private final Shooter shooter;
	private final double speed;
	private final boolean inverted;

	public FeedShooterCommand(Feeder feeder, Shooter shooter, boolean inverted) {
		this.feeder = feeder;
		this.shooter = shooter;
		this.speed = 0.5;
		this.inverted = inverted;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(feeder);
	}
	
	public FeedShooterCommand(Feeder feeder, Shooter shooter, double speed, boolean inverted) {
		this.feeder = feeder;
		this.shooter = shooter;
		this.speed = speed;
		this.inverted = inverted;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// if(shooter.getSetpoint() == 0 && !inverted) return;
		feeder.feedShooter(speed, inverted);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		feeder.feedShooter(0, inverted);
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
