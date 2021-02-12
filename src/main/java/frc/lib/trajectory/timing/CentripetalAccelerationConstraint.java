package frc.lib.trajectory.timing;

import frc.lib.geometry.PoseWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<PoseWithCurvature> {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final PoseWithCurvature state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final PoseWithCurvature state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
