/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
* An example command that uses an example subsystem.
*/
public class MoveConveyorCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Conveyor conveyor;
	private final Feeder feeder;
	private final Shooter shooter;
	private final double speed;
	private final boolean inverted;
	
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public MoveConveyorCommand(Conveyor conveyor, Feeder feeder, Shooter shooter, double speed, boolean inverted) {
		this.conveyor = conveyor;
		this.feeder = feeder;
		this.shooter = shooter;
		this.speed = speed;
		this.inverted = inverted;
		// Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(intake);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("Conveyor MOVE");
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		conveyor.rollConveyor(speed, inverted);
		if(shooter.getSetpoint() == 0) {
			feeder.feedShooter(0.1, true);
		}
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		conveyor.rollConveyor(0, false);
		System.out.println("Conveyor MOVE COMMAND | END");
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
