package frc.lib.trajectory;

import frc.lib.geometry.Pose;
import frc.lib.geometry.Twist;

public interface IPathFollower {
    public Twist steer(Pose current_pose);

    public boolean isDone();
}
