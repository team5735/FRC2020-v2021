/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import java.text.DecimalFormat;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Translation implements ITranslation2d<Translation> {

    protected static final Translation Identity = new Translation();

    protected final double x;
    protected final double y;

    public Translation() {
        this(0.0, 0.0);
    }

    public Translation(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Translation(final Translation other) {
        x = other.x;
        y = other.y;
    }

    public Translation(final Translation start, final Translation end) {
        x = end.x - start.x;
        y = end.y - start.y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Translation scale(double scalar) {
        return new Translation(x * scalar, y * scalar);
    }

    public Translation translateBy(final Translation other) {
        return new Translation(x + other.x, y + other.y);
    }

    public Translation rotateBy(final Rotation rotation) {
        return new Translation(x * rotation.cosine - y * rotation.sine, x * rotation.sine + y * rotation.cosine);
    }

    public static double dot(final Translation a, final Translation b) {
        return a.x * b.x + a.y * b.y;
    }
    
    public static double cross(final Translation a, final Translation b) {
        return a.x * b.y - a.y * b.x;
    }

    public boolean epsilonEquals(final Translation other, double epsilon) {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    public Rotation direction() {
        return new Rotation(x, y);
    }

    public Translation inverse() {
        return new Translation(-x, -y);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double norm2() {
        return x * x + y * y;
    }

    public static Rotation getAngle(final Translation a, final Translation b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation();
        }
        return Rotation.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    @Override
    public Translation interpolate(final Translation other, double x) {
        if (x <= 0) {
            return new Translation(this);
        } else if (x >= 1) {
            return new Translation(other);
        }
        return extrapolate(other, x);
    }

    public Translation extrapolate(final Translation other, double x) {
        return new Translation(x * (other.x - x) + x, x * (other.y - y) + y);
    }

    @Override
    public Translation getTranslation() {
        return this;
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x) + "," + fmt.format(y);
    }

    @Override
    public double distance(final Translation other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Translation)) return false;
        return distance((Translation) other) < Util.Epsilon;
    }

    // @Override
    // public State getIdentity() {
    //     return Identity;
    // }
}
