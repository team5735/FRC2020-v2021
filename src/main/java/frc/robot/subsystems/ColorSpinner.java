/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class ColorSpinner extends SubsystemBase {
  private TalonSRX talon;
  private double ratio, deltaCompensation;

  private static final double tolerance = 30;
  private static final int DRIVE_DIAMETER = 4;

  public ColorSpinner() {
    talon = new TalonSRX(RobotConstants.COLOR_SPINNER_MOTOR_ID);
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    talon.setInverted(false);
    talon.configContinuousCurrentLimit(39);
    talon.enableCurrentLimit(true);
    talon.configMotionCruiseVelocity(4095);
    talon.configMotionAcceleration(1023);

    talon.selectProfileSlot(0, 0);
    talon.config_kP(0, 1);
    talon.config_kI(0, 0);
    talon.config_kD(0, 10);
    talon.config_kF(0, 0.5);

    ratio = 32.0 / DRIVE_DIAMETER * 4.0;
    deltaCompensation = 1.075;

    SmartDashboard.putNumber("Revolutions", 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    talon.setSelectedSensorPosition(0);
  }

  public void spin(double revolutions) {
    talon.set(ControlMode.MotionMagic, revolutionsToEncoderTicks(revolutions));
  }

  public void stop() {
    talon.set(ControlMode.PercentOutput, 0);
  }

  public double getSelectedSensorPosition() {
    return talon.getSelectedSensorPosition();
  }

  public boolean withinTolerance(double revolutions) {
    return Math.abs(getSelectedSensorPosition() - revolutionsToEncoderTicks(revolutions)) < tolerance;
  }

  public double revolutionsToEncoderTicks(double revolutions){
    return revolutions * 1024.0 * ratio * deltaCompensation;
  }

}
