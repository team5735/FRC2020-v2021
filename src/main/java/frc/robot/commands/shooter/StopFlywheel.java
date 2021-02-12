/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopFlywheel extends CommandBase {

  private Shooter shooter;

  /**
   * Creates a new StopFlywheel.
   */
  public StopFlywheel(Shooter shooter) {
		this.shooter = shooter;
		
		// Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(this.shooter);
	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.slowDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.slowDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.slowDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
