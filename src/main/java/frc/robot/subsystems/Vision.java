/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimeLight;
import frc.robot.constants.RobotConstants;

public class Vision extends SubsystemBase {

	private LimeLight limelight;
	private boolean trackingMode = false;
	private boolean hasValidTarget = false;

	public enum LimelightPipeline {
		TESTING(0), NORTH_SHORE(1), GREATER_BOSTON(2);

		private final int value;

		private LimelightPipeline(int id) {
			this.value = id;
		}

		public int getValue() {
			return value;
		}
	};

	public Vision() {
		limelight = new LimeLight();
		limelight.setPipeline(0);// LimelightPipeline.TESTING.getValue());
		disableTracking();

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	@Override
	public void periodic() {
		hasValidTarget = isTargetFound();
	}

	// distanceFromCamera = (RobotConstants.TARGETHEIGHTFROMGROUND -
	// RobotConstants.CAMERAHEIGHTFROMGROUND)
	// / Math.tan(Math.toRadians(ty) + RobotConstants.CAMERAANGLEFROMPARALLEL);

	/**
	 * @return Horizontal distance to target, in meters
	 * distance = height from camera to target / tan(angle to parallell from camera)
	 */
	public double getDistanceToTarget() {
		if (!isTrackingEnabled()) {
			enableTracking();
		}
		boolean gotDistance = false;

		double heightDiff = RobotConstants.TARGET_HEIGHTFROMGROUND - RobotConstants.CAMERA_HEIGHTFROMGROUND;
		double distance = 0;

		while (!gotDistance) {
			double yAngleToTarget = Units.degreesToRadians(limelight.getdegVerticalToTarget()); // radians
			distance = heightDiff / Math.tan(RobotConstants.CAMERA_ANGLEFROMPARALLEL + yAngleToTarget); // meters
			if (distance > 1) {
				gotDistance = true;
			}
		}

		// disableTracking();

		System.out.println("Distance to Target: " + distance);
		SmartDashboard.putNumber("Distance to Target (m)", distance);

		return distance;
	}

	public void enableTracking() {
		System.out.println("ENABLED TRACKING");
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		trackingMode = true;
	}

	public void disableTracking() {
		System.out.println("DISABLED TRACKING");
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
		trackingMode = false;
	}
	
	public boolean isTrackingEnabled() {
		return trackingMode;
	}

	public boolean isTargetFound() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) > 0;
	}

	public boolean hasValidTarget() {
		SmartDashboard.putBoolean("Has Target", hasValidTarget);
		return hasValidTarget;
	}

	public LimeLight getLimelight() {
		return limelight;
	}

	public double getTX() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
	}

	public double getTY() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
	}
}
