/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Pose implements IPose2d<Pose>{

    public static final Pose Identity = new Pose();

    protected final Translation translation;
    protected final Rotation rotation;

    public Pose() {
        this(new Translation(), new Rotation());
    }

    public Pose(double x, double y, Rotation rotation) {
        this(new Translation(x, y), rotation);

    }

    public Pose(final Pose other) {
        translation = new Translation(other.translation);
        rotation = new Rotation(other.rotation);
    }

    public Pose(Translation translation, Rotation rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public static Pose fromTranslation(final Translation translation) {
        return new Pose(translation, new Rotation());
    }

    public static Pose fromRotation(final Rotation rotation) {
        return new Pose(new Translation(), rotation);
    }

    public Pose getPose() {
        return this;
    }

    public boolean isColinear(final Pose other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public Pose transformBy(final Pose other) {
        return new Pose(translation.translateBy(other.translation.rotateBy(rotation)),
                rotation.rotateBy(other.rotation));
    }

    public Pose inverse() {
        Rotation rotation_inverted = rotation.inverse();
        return new Pose(translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public Pose normal() {
        return new Pose(translation, rotation.normal());
    }

    public static Pose exp(final Twist delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < Util.Epsilon) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose(new Translation(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation(cos_theta, sin_theta));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist log(final Pose transform) {
        final double dtheta = transform.getRotation().radians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cosine - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < Util.Epsilon) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sine / cos_minus_one);
        }
        final Translation translation_part = transform.getTranslation()
                .rotateBy(new Rotation(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist(translation_part.x(), translation_part.y(), dtheta);
    }

    @Override
    public Pose interpolate(final Pose other, double x) {
        if (x <= 0) {
            return new Pose(this);
        } else if (x >= 1) {
            return new Pose(other);
        }
        final Twist twist = Pose.log(inverse().transformBy(other));
        return transformBy(Pose.exp(twist.scaled(x)));
    }

    public Translation intersection(final Pose other) {
        final Rotation other_rotation = other.getRotation();
        if (rotation.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation.cosine()) < Math.abs(other_rotation.cosine())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    private static Translation intersectionInternal(final Pose a, final Pose b) {
        final Rotation a_r = a.getRotation();
        final Rotation b_r = b.getRotation();
        final Translation a_t = a.getTranslation();
        final Translation b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sine() - a_r.cosine() * tan_b);
        if (Double.isNaN(t)) {
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    @Override
    public Translation getTranslation() {
        return translation;
    }

    @Override
    public Rotation getRotation() {
        return rotation;
    }

    @Override
    public double distance(final Pose other) {
        return Pose.log(inverse().transformBy(other)).norm();
    }

    @Override
    public String toCSV() {
        return translation.toCSV() + "," + rotation.toCSV();
    }

    @Override
    public Pose mirror() {
        return new Pose(new Translation(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }

    @Override
    public String toString() {
        return "x: " + translation.x + " y: " + translation.y + " r: " + rotation.degrees();
    }
}
