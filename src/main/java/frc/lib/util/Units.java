package frc.lib.util;

import frc.robot.constants.RobotConstants;

public class Units {
    
    /* Distance Conversion */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public static double feetToMeters(double feet) {
        return inchesToMeters(feet * 12.0);
    }

    public static double metersToFeet(double meters) {
        return metersToInches(meters) / 12.0;
    }

    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /* Sensor Conversion */
    public static double dtRotationsToMeters(double rotations) {
        return rotations * (RobotConstants.DT_WHEEL_DIAMETER * Math.PI);
    }

    public static double dtMetersToRotations(double meters) {
        return meters / (RobotConstants.DT_WHEEL_DIAMETER * Math.PI);
    }

    public static double normalMetersToRotations(double meters) {
        return meters / (RobotConstants.DT_WHEEL_DIAMETER * Math.PI);
    }

    public static double dtRotationsToTicks(double rotations) {
        return rotations * RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV;
    }

    public static double dtTickstoRotations(double ticks) {
        return ticks / RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV;
    }

    public static double dtRpmToTicks(double rpm) {
        return rpm * RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV / 60.0 / 10.0;
    }

    public static double normalRpmToTicks(double rpm) {
        return rpm * RobotConstants.ENCODER_TICKS_PER_NORMAL_WHEEL_REV / 60.0 / 10.0;
    }

    public static double dtMetersPerSecondToRpm(double metersPerSec) {
        return dtMetersToRotations(metersPerSec) * 60.0;
    }
    
    public static double normalMetersPerSecondToRpm(double metersPerSec) {
        return normalMetersToRotations(metersPerSec) * 60.0;
    }
    
    public static double radiansPerSecondToRPM(double radians) {
        return radians / (Math.PI * 2.0) * 60.0;
    }

    public static double dtRadiansPerSecondToTicks(double radians) {
        return radians / (Math.PI * 2.0) * RobotConstants.ENCODER_TICKS_PER_DT_WHEEL_REV / 10.0;
    }

}