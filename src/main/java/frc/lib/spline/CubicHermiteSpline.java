package frc.lib.spline;

import frc.lib.geometry.Pose;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Translation;

/**
 * Temporary spline for testing
 */
public class CubicHermiteSpline extends Spline {
    private final double ax, bx, cx, dx, ay, by, cy, dy;

    public CubicHermiteSpline(Pose p0, Pose p1) {
        double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
        double scale = 2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cosine() * scale;
        dx1 = p1.getRotation().cosine() * scale;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sine() * scale;
        dy1 = p1.getRotation().sine() * scale;
        ax = dx0 + dx1 + 2 * x0 - 2 * x1;
        bx = -2 * dx0 - dx1 - 3 * x0 + 3 * x1;
        cx = dx0;
        dx = x0;
        ay = dy0 + dy1 + 2 * y0 - 2 * y1;
        by = -2 * dy0 - dy1 - 3 * y0 + 3 * y1;
        cy = dy0;
        dy = y0;
    }

    @Override
    public Translation getPoint(double t) {
        final double x = t * t * t * ax + t * t * bx + t * cx + dx;
        final double y = t * t * t * ay + t * t * by + t * cy + dy;
        return new Translation(x, y);
    }

    @Override
    public Rotation getHeading(double t) {
        final double dx = 3 * t * t * ax + 2 * t * bx + cx;
        final double dy = 3 * t * t * ay + 2 * t * by + cy;
        return new Rotation(dx, dy);
    }

    @Override
    public double getVelocity(double t) {
        // TODO implement this
        return 1.0;
    }

    @Override
    public double getCurvature(double t) {
        final double dx = 3 * t * t * ax + 2 * t * bx + cx;
        final double dy = 3 * t * t * ay + 2 * t * by + cy;
        final double ddx = 6 * t * ax + 2 * bx;
        final double ddy = 6 * t * ay + 2 * by;
        return (dx * ddy - dy * ddx) / ((dx * dx + dy * dy) * Math.sqrt(dx * dx + dy * dy));
    }

    @Override
    public double getDCurvature(double t) {
        // TODO implement this
        return 0.0;
    }
}
