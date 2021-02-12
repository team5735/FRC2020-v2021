/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Winch extends SubsystemBase {
  private final CANSparkMax winchMaster;

  /**
   * Creates a new Climber.
   */
  public Winch() {
    winchMaster = new CANSparkMax(RobotConstants.WINCH_ID, MotorType.kBrushless);
    winchMaster.restoreFactoryDefaults();
    winchMaster.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveWinch(double output) {
    winchMaster.set(output);
  }

}
