package frc.lib.trajectory.timing;

import frc.lib.geometry.ITranslation2d;
import frc.lib.geometry.Translation;

public class VelocityLimitRegionConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
    protected final Translation min_corner_;
    protected final Translation max_corner_;
    protected final double velocity_limit_;

    public VelocityLimitRegionConstraint(Translation min_corner, Translation max_corner, double velocity_limit) {
        min_corner_ = min_corner;
        max_corner_ = max_corner;
        velocity_limit_ = velocity_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        final Translation translation = state.getTranslation();
        if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
                translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y()) {
            return velocity_limit_;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state,
                                                                     double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }

}
