/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helper;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class HDriveHelper {

	public static double ANGULAR_PERCENTAGE = 0.2;//0.33;

    public static DriveSignal xyLockedDrive(double x, double y) {
        return HDrive(x, y, 0, false);
    }

    /**
     * 
     * @param forward
     * @param normal
     * @param angular
     * @param fc  Is Field Centric 
     * @return DriveSignal
     */
    public static DriveSignal HDrive(double forward, double normal, double angular, boolean fc) {
		double leftPercentage = forward * (1 - ANGULAR_PERCENTAGE) + angular * ANGULAR_PERCENTAGE;
		double rightPercentage = forward * (1 - ANGULAR_PERCENTAGE) - angular * ANGULAR_PERCENTAGE;
        double normalPercentage = normal * (1 - ANGULAR_PERCENTAGE);
        
        // leftPercentage *= (fc ? RobotConstants.MAX_VELOCITY_NORMAL_TICKS : RobotConstants.MAX_VELOCITY_DT_TICKS);
        // rightPercentage *= (fc ? RobotConstants.MAX_VELOCITY_NORMAL_TICKS : RobotConstants.MAX_VELOCITY_DT_TICKS);
        // normalPercentage *= RobotConstants.MAX_VELOCITY_NORMAL_TICKS;
        
        SmartDashboard.putNumber("Left Velocity Setpoint", leftPercentage);
        SmartDashboard.putNumber("Right Velocity Setpoint", rightPercentage);

        // return new DriveSignal(ControlMode.Velocity, leftPercentage, rightPercentage, normalPercentage);
        return new DriveSignal(ControlMode.PercentOutput, leftPercentage, rightPercentage, normalPercentage);
    }

    /**
     * Drive field-centric with respect to current gyro angle
     * @param forwardPercent
     * @param sidewaysPercent
     * @param angularPercent
     * @param currentAngle
     * @return DriveSignal
     */
    public static DriveSignal HdriveFieldCentric(double forwardPercent, double sidewaysPercent, double angularPercent, double currentAngle) {
		double angleRad = Math.toRadians(currentAngle);
		double modifiedForward = forwardPercent * Math.cos(angleRad) + sidewaysPercent * Math.sin(-angleRad); // %
		double modifiedNormal = forwardPercent * Math.sin(angleRad) + sidewaysPercent * Math.cos(angleRad); // %
        
        // modifiedForward *= RobotConstants.MAX_VELOCITY_NORMAL_TICKS;
        // modifiedNormal *= RobotConstants.MAX_VELOCITY_NORMAL_TICKS;
        
        return HDrive(modifiedForward, modifiedNormal, angularPercent, true);
        
		// return new DriveSignal(ControlMode.Velocity, modifiedForward, modifiedForward, modifiedNormal);
	}

    /* Uses poses

    // translation is direction of travel with magnitute
    // rotation is rotating 

    public static DriveSignal xyLockedDrive(Translation translation) {
        return hDrive(translation, Rotation.Identity);
    }

    // ticksper100ms
    public static DriveSignal hDrive(Translation translation, Rotation rotation) {
        double arcLength = rotation.radians() * RobotConstants.DRIVETRAIN_TRACK_WIDTH / 2.0; // in inches/100ms
        double left = translation.y()  - Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double right = translation.y() + Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double normal = translation.x();

        // System.out.println("[D] Forward Percentage: " + leftPercentage);
        // System.out.println("[D] Normal Percentage: " + sidewaysPercentage);

        return new DriveSignal(left, right, normal);
    }

    public static DriveSignal correctField(Translation translation, Rotation rotation, double fieldAngle) {
        return hDrive(translation.rotateBy(Rotation.fromDegrees(fieldAngle)), rotation);
    }
    
	 * Drive by providing velocities in sensor ticks / 100 ms
	 * @param controlMode
	 * @param forwardVelocity
	 * @param sidewaysVelocity
	 * @param angularVelocity
     * 
	public void driveTurn(ControlMode controlMode, double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
		double leftPercentage = forwardVelocity * (1 - ANGULAR_PERCENTAGE) + angularVelocity * ANGULAR_PERCENTAGE;
		double rightPercentage = forwardVelocity * (1 - ANGULAR_PERCENTAGE) - angularVelocity * ANGULAR_PERCENTAGE;
		double sidewaysPercentage = sidewaysVelocity * (1 - ANGULAR_PERCENTAGE);
		
		drive(controlMode, new DriveSignal(leftPercentage, rightPercentage, sidewaysPercentage));
    }
    
    public void driveFieldCentric(double forwardPercent, double sidewaysPercent, double angularPercent, double currentAngle) {
		double angleRad = Math.toRadians(currentAngle);
		double modifiedForward = forwardPercent * Math.cos(angleRad) + sidewaysPercent * Math.sin(-angleRad); // %
		double modifiedSideways = forwardPercent * Math.sin(angleRad) + sidewaysPercent * Math.cos(angleRad); // %
		drivePercent(ControlMode.Velocity, modifiedForward, modifiedSideways, angularPercent, RobotConstants.MAX_VELOCITY_NORMAL_TICKS);
	}

    */
}
