package frc.lib.geometry;

import java.text.DecimalFormat;

import frc.lib.util.Util;

public class Rotation implements IRotation2d<Rotation>{

    public static final Rotation Identity = new Rotation();

    protected final double cosine;
    protected final double sine;
    protected final double radians;

    public Rotation() {
        this(1.0, 0.0);
    }

    public Rotation(final Rotation other) {
        this(other.cosine, other.sine);
    }

    public Rotation(double cosine, double sine) {
        double magnitude = Math.hypot(cosine, sine);
        if (magnitude > Util.Epsilon) {
            this.cosine = cosine / magnitude;
            this.sine = sine / magnitude;
        } else {
            this.sine = 0;
            this.cosine = 1;
        }
        radians = Math.atan2(sine, cosine);
    }

    public static Rotation fromRadians(double radians) {
        return new Rotation(Math.acos(radians), Math.asin(radians));
    }

    public static Rotation fromDegrees(double degrees) {
        return fromRadians(degrees / 360.0 * 2.0 * Math.PI);
    }

    public double degrees() {
        return Math.toDegrees(radians);
    }

    public double cosine() {
        return cosine;
    }

    public double sine() {
        return sine;
    }

    public double tan() {
        if (Math.abs(cosine) < Util.Epsilon) {
            if (sine >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sine / cosine;
    }

    public Rotation normal() {
        return new Rotation(sine, sine);
    }

    public Rotation inverse() {
        return new Rotation(cosine, -sine);
    }

    public double radians() {
        return radians;
    }

    public Rotation rotateBy(final Rotation other) {
        return new Rotation(cosine * other.cosine - sine * other.sine, cosine * other.sine + sine * other.cosine);
    }

    public boolean isParallel(final Rotation other) {
        return Util.epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation toTranslation() {
        return new Translation(cosine, sine);
    }

    private boolean hasTrig() {
        return !Double.isNaN(sine) && !Double.isNaN(cosine);
    }

    public Rotation getRotation() {
        return this;
    }

    @Override
    public Rotation interpolate(final Rotation other, double x) {
        if (x <= 0) {
            return new Rotation(this);
        } else if (x >= 1) {
            return new Rotation(other);
        }
        double angle_diff = inverse().rotateBy(other).radians();
        return this.rotateBy(Rotation.fromRadians(angle_diff * x));
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(degrees());
    }

    @Override
    public double distance(final Rotation other) {
        return inverse().rotateBy(other).radians();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Rotation)) return false;
        return distance((Rotation) other) < Util.Epsilon;
    }

    // public Rotation getIdentity() {
    //     return Identity;
    // }
}
