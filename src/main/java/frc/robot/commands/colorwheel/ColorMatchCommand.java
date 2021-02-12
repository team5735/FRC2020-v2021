/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorwheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.ColorSpinner;

/**
 * An example command that uses an example subsystem.
 */
public class ColorMatchCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorSpinner colorSpinner;
  private final ColorMatcher colorMatcher;
  private String expectedColor = "";

  /**
   * Creates a new ColorMatchCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ColorMatchCommand(ColorSpinner subsystem, ColorMatcher colorMatcher) {
    this.colorSpinner = subsystem;
    this.colorMatcher = colorMatcher;

    expectedColor = DriverStation.getInstance().getGameSpecificMessage();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(colorMatcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSpinner.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (expectedColor == null || expectedColor.equalsIgnoreCase(""))
      return;

    String color = colorMatcher.getColor();
    while (!color.startsWith(expectedColor)) {
      colorSpinner.spin(1.0/8.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSpinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (expectedColor == null || expectedColor.equalsIgnoreCase(""))
      return true;

      String color = colorMatcher.getColor();
    return color.startsWith(expectedColor);
  }

}
