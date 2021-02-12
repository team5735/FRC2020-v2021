/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Telescope;

/**
* An example command that uses an example subsystem.
*/
public class ElevatorMoveCommand extends CommandBase {
	private final Telescope telescope;
	private final boolean inverted;
	
	public ElevatorMoveCommand(Telescope telescope, boolean inverted) {
		this.telescope = telescope;
		this.inverted = inverted;
		addRequirements(telescope);
	}
	
	@Override
	public void initialize() {
	}
	
	@Override
	public void execute() {
		telescope.moveElevator((inverted ? -1 : 1) * 0.45);
	}
	
	@Override
	public void end(boolean interrupted) {
		telescope.moveElevator(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
