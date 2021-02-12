package frc.lib.spline;

import frc.lib.geometry.Pose;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Translation;

public abstract class Spline {
    public abstract Translation getPoint(double t);

    public abstract Rotation getHeading(double t);

    public abstract double getCurvature(double t);

    // dk/ds
    public abstract double getDCurvature(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    public Pose getPose(double t) {
        return new Pose(getPoint(t), getHeading(t));
    }

    public PoseWithCurvature getPoseWithCurvature(double t) {
        return new PoseWithCurvature(getPose(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
    }

    // TODO add toString
    // public abstract String toString();
}
