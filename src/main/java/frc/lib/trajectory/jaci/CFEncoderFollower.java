package frc.lib.trajectory.jaci;

import jaci.pathfinder.Trajectory;

public interface CFEncoderFollower {
    public void setTrajectory(Trajectory traj);
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka);
    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter);
    public void reset();
    public double calculate(int encoder_tick);
    public double getHeading();
    public Trajectory.Segment getSegment();
    public boolean isFinished();
}