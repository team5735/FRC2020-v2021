/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;

public class Shooter extends SubsystemBase {

	// private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> DistanceToRPM;

	private CANSparkMax neoMaster, neoSlave;
	private CANPIDController flywheelPIDController;

	private double speedSetpoint;

	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {

		neoMaster = new CANSparkMax(RobotConstants.FLYWHEEL_MASTER_ID, MotorType.kBrushless);
		neoMaster.restoreFactoryDefaults();
		neoMaster.setInverted(false);
		neoMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);


		neoSlave = new CANSparkMax(RobotConstants.FLYWHEEL_SLAVE_ID, MotorType.kBrushless);
		neoSlave.restoreFactoryDefaults();
		neoSlave.follow(neoMaster, true);
		neoSlave.enableSoftLimit(SoftLimitDirection.kReverse, true);

		neoMaster.getEncoder().setVelocityConversionFactor(RobotConstants.FLYWHEEL_PULLEY_RATIO);

		flywheelPIDController = neoMaster.getPIDController();

		flywheelPIDController.setP(RobotConstants.FLYWHEEL_kP, 0);
		flywheelPIDController.setI(RobotConstants.FLYWHEEL_kI, 0);
		flywheelPIDController.setD(RobotConstants.FLYWHEEL_kD, 0);
		flywheelPIDController.setFF(RobotConstants.FLYWHEEL_kF, 0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Shooter Speed (RPM)", getSpeed());
	}

	public void moveExplicit(double percent) {
		neoMaster.set(percent);
	}

	public void setSpeed(double rpm) {
		double speed = rpm;
		if (rpm > RobotConstants.FLYWHEEL_MAX_SPEED)
			speed = RobotConstants.FLYWHEEL_MAX_SPEED;
		if (rpm < RobotConstants.FLYWHEEL_MIN_SPEED)
			speed = RobotConstants.FLYWHEEL_MIN_SPEED;
		speedSetpoint = speed;
		flywheelPIDController.setReference(speed, ControlType.kVelocity);
	}

	public double getSpeed() {
		return neoMaster.getEncoder().getVelocity();
	}

	public boolean atSpeed(double threshold) {
		// System.out.println("Shooter Speed (RPM): " + getSpeed());
		return Util.deadband(speedSetpoint - getSpeed(), threshold) == 0;
	}

	public void slowDown() {
		neoMaster.set(0);
		speedSetpoint = 0;
	}

	public double getSetpoint() {
		return speedSetpoint;
	}

	/**
	 * Function to convert a distance into flywheel speed
	 * 
	 * @param distance Horizontal distance to target, in meters
	 * @return Flywheel speed, in RPM
	 */
	public double getSpeedFromDistance(double x) {
		// return DistanceToRPM.getInterpolated(new InterpolatingDouble(distance)).value;
		// return 52.4826534001*(x*x) - 286.6534831746*(x) + 3836.4028991883;
		// return 59.4992519392*(x*x) - 357.2897939639*(x) + 3969.1694854952;

		System.out.println("X: " + x);
		SmartDashboard.putNumber("X value given", x);
		return 16.1564869793*(x*x) - 22.0935926212*(x) + 3365.2543639255;
	}

}
