/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import frc.robot.commands.drivetrain.DriveJoystick;
import frc.robot.constants.RobotConstants;

public class Drivetrain extends SubsystemBase{
	
	private TalonFX leftMaster, rightMaster, leftFollower, rightFollower, normalMaster;
	public static TalonSRX gyroHost;
	private PigeonIMU gyro;

	private double previousGyroAngle = 0.0; // previous gyro angle before turn to target
		
	public enum DriveMode {
		STATIC_DRIVE, FIELD_CENTRIC, DISABLED
	};
	
	private DriveMode driveMode = DriveMode.STATIC_DRIVE; //TODO Change to Field Centric
	
	public Drivetrain() {
		leftMaster = new TalonFX(RobotConstants.LEFT_MASTER_ID);
		leftMaster.configFactoryDefault();
		leftMaster.setInverted(true);
		leftMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		leftMaster.config_kP(0, RobotConstants.LEFT_kP);
		leftMaster.config_kI(0, RobotConstants.LEFT_kI);
		leftMaster.config_kD(0, RobotConstants.LEFT_kD);
		leftMaster.config_kF(0, RobotConstants.LEFT_kF);
		
		leftFollower = new TalonFX(RobotConstants.LEFT_SLAVE_ID);
		leftFollower.configFactoryDefault();
		leftFollower.follow(leftMaster);
		leftFollower.setInverted(true);
		leftFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		
		rightMaster = new TalonFX(RobotConstants.RIGHT_MASTER_ID);
		rightMaster.configFactoryDefault();
		rightMaster.setInverted(false);
		rightMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		rightMaster.config_kP(0, RobotConstants.RIGHT_kP);
		rightMaster.config_kI(0, RobotConstants.RIGHT_kI);
		rightMaster.config_kD(0, RobotConstants.RIGHT_kD);
		rightMaster.config_kF(0, RobotConstants.RIGHT_kF);
		
		rightFollower = new TalonFX(RobotConstants.RIGHT_SLAVE_ID);
		rightFollower.configFactoryDefault();
		rightFollower.follow(rightMaster);
		rightFollower.setInverted(false);
		rightFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		
		normalMaster = new TalonFX(RobotConstants.NORMAL_ID);
		normalMaster.configFactoryDefault();
		normalMaster.setInverted(false);
		normalMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		normalMaster.config_kP(0, RobotConstants.NORMAL_kP);
		normalMaster.config_kI(0, RobotConstants.NORMAL_kI);
		normalMaster.config_kD(0, RobotConstants.NORMAL_kD);
		normalMaster.config_kF(0, RobotConstants.NORMAL_kF);
		
		gyroHost = new TalonSRX(RobotConstants.GYRO_TALON_HOST_ID);
		gyroHost.configFactoryDefault();
		gyro = new PigeonIMU(gyroHost);
		
		CommandScheduler.getInstance().setDefaultCommand(this, new DriveJoystick(this));
	}
	
	/**
	 * Drive with drive signal
	 * @param driveSignal
	 */
	public void drive(DriveSignal driveSignal) {
		drive(driveSignal.getControlMode(), driveSignal.getLeft(), driveSignal.getRight(), driveSignal.getNormal());
	}

	/**
	 * Drive Explicitly
	 * @param controlMode
	 * @param left
	 * @param right
	 * @param normal
	 */
	public void drive(ControlMode controlMode, double left, double right, double normal) {
		leftMaster.set(controlMode, left);
		rightMaster.set(controlMode, right);
		normalMaster.set(controlMode, normal);
		// System.out.println("L: " + leftMaster.getSelectedSensorVelocity() + ", R: " + rightMaster.getSelectedSensorVelocity() + ", N: " + normalMaster.getSelectedSensorVelocity());
	}

	
	public void reset() {
		zeroSensors();
	}
	
	public void zeroSensors() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);
		gyro.setFusedHeading(0);
	}
	
	public int getLeftSidePosition() {
		return (int) (leftMaster.getSelectedSensorPosition() * RobotConstants.DRIVETRAIN_GEAR_RATIO);
	}
	
	public int getRightSidePosition() {
		return (int) (rightMaster.getSelectedSensorPosition() * RobotConstants.DRIVETRAIN_GEAR_RATIO);
	}
	
	public double getLeftVelocity() {
		return leftMaster.getSelectedSensorVelocity(); 
	}
	
	public double getRightVelocity() {
		return rightMaster.getSelectedSensorVelocity(); 
	}
	
	public double getGyroAngle() {
		return gyro.getFusedHeading();
	}
	
	public void resetGyroAngle() {
		gyro.setFusedHeading(0);
	}

	public void setPreviousGyroAngle() {
		previousGyroAngle = getGyroAngle();
	}

	public double getPreviousGyroAngle() {
		return previousGyroAngle;
	}
	
	public void setDriveMode(DriveMode driveMode) {
		this.driveMode = driveMode;
	}
	
	public DriveMode getDriveMode() {
		return driveMode;
	}
}
