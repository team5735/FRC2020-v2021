/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class ChangeDriveMode extends CommandBase {
/*
  private Drivetrain drivetrain;

  public ChangeDriveMode(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    addRequirements((Subsystem) drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drivetrain.setDriveMode((drivetrain.getDriveMode() == DriveMode.STATIC_DRIVE) ? DriveMode.FIELD_CENTRIC : DriveMode.STATIC_DRIVE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVE MODE: " + drivetrain.getDriveMode().toString());
    SmartDashboard.putString("Drive Mode", drivetrain.getDriveMode().toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  */
}
