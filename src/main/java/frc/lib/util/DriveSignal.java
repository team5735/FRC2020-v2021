package frc.lib.util;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * A drivetrain command consisting of the left, right, and normal motor settings
 * inches per second2
 */
public class DriveSignal {

    protected ControlMode controlMode;
    protected double left;
    protected double right;
    protected double normal; // right is positive

    public DriveSignal(ControlMode controlMode, double left, double right) {
        this(controlMode, left, right, 0);
    }

    public DriveSignal(ControlMode controlMode, double left, double right, double normal) {
        this.controlMode = controlMode;
        this.left = left;
        this.right = right;
        this.normal = normal;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(ControlMode.PercentOutput, 0, 0, 0);

    public ControlMode getControlMode() {
        return controlMode;
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getNormal() {
        return normal;
    }

    public void scale(double factor) {
        scaleLeft(factor);
        scaleRight(factor);
        scaleNormal(factor);
    }

    public void scaleLeft(double factor) {
        left *= factor;
    }

    public void scaleRight(double factor) {
        right *= factor;
    }

    public void scaleNormal(double factor) {
        normal *= factor;
    }

    @Override
    public String toString() {
        return "L: " + left + ", R: " + right + " C:" + normal;
    }
}
