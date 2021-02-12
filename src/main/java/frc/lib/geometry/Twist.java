/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Twist {
    public static final Twist identity = new Twist(0.0, 0.0, 0.0);
    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!
    public Twist(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public static final Twist identity() {
        return identity;
    }

    public Twist scaled(double scale) {
        return new Twist(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.Epsilon && norm() < Util.Epsilon)
            return 0.0;
        return dtheta / norm();
    }

    public Translation getTranslation() {
        return new Translation(dx, dy);
    }

    public Rotation getRotation() {
        return Rotation.fromRadians(dtheta);
    }

    // @Override
    // public String toString() {
    //     final DecimalFormat fmt = new DecimalFormat("#0.000");
    //     return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    // }
}
