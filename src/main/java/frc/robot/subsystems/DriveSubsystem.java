package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.Units;
import frc.robot.constants.RobotConstants;

public class DriveSubsystem extends SubsystemBase {
	private TalonFX leftMaster, rightMaster, leftFollower, rightFollower, normalMaster;

	// The gyro sensor
	private final PigeonIMU m_gyro = new PigeonIMU(new TalonSRX(RobotConstants.GYRO_TALON_HOST_ID));
	
	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;
	
	/**
	* Creates a new DriveSubsystem.
	*/
	public DriveSubsystem() {
		leftMaster = new TalonFX(RobotConstants.LEFT_MASTER_ID);
		leftMaster.configFactoryDefault();
		leftMaster.setInverted(true);
		leftMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		leftMaster.config_kP(0, RobotConstants.LEFT_kP);
		leftMaster.config_kI(0, RobotConstants.LEFT_kI);
		leftMaster.config_kD(0, RobotConstants.LEFT_kD);
		leftMaster.config_kF(0, RobotConstants.LEFT_kF);
		
		leftFollower = new TalonFX(RobotConstants.LEFT_SLAVE_ID);
		leftFollower.configFactoryDefault();
		leftFollower.follow(leftMaster);
		leftFollower.setInverted(true);
		leftFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		
		rightMaster = new TalonFX(RobotConstants.RIGHT_MASTER_ID);
		rightMaster.configFactoryDefault();
		rightMaster.setInverted(false);
		rightMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		rightMaster.config_kP(0, RobotConstants.RIGHT_kP);
		rightMaster.config_kI(0, RobotConstants.RIGHT_kI);
		rightMaster.config_kD(0, RobotConstants.RIGHT_kD);
		rightMaster.config_kF(0, RobotConstants.RIGHT_kF);
		
		rightFollower = new TalonFX(RobotConstants.RIGHT_SLAVE_ID);
		rightFollower.configFactoryDefault();
		rightFollower.follow(rightMaster);
		rightFollower.setInverted(false);
		rightFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);

		resetEncoders();
		m_odometry = new DifferentialDriveOdometry(getGyroRotation());
	}
	
	@Override
	public void periodic() {
		double[] distance = getDtDistance();
		// Update the odometry in the periodic block
		m_odometry.update(getGyroRotation(), distance[0], distance[1]);
	}
	
	/**
	* Returns the currently-estimated pose of the robot.
	*
	* @return The pose.
	*/
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}
	
	/**
	* Returns the current wheel speeds of the robot.
	*
	* @return The current wheel speeds.
	*/
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		double[] vel = getDtVelocity();
		return new DifferentialDriveWheelSpeeds(vel[0], vel[1]);
	}
	
	/**
	* Resets the odometry to the specified pose.
	*
	* @param pose The pose to which to set the odometry.
	*/
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, getGyroRotation());
	}

	/**
	 * Drive with drive signal
	 * @param driveSignal
	 */
	public void drive(DriveSignal driveSignal) {
		drive(driveSignal.getControlMode(), driveSignal.getLeft(), driveSignal.getRight(), driveSignal.getNormal());
	}

	/**
	 * Drive Explicitly
	 * @param controlMode
	 * @param left
	 * @param right
	 * @param normal
	 */
	public void drive(ControlMode controlMode, double left, double right, double normal) {
		leftMaster.set(controlMode, left);
		rightMaster.set(controlMode, right);
		normalMaster.set(controlMode, normal);
//THIS IS AWESOME!!!! --> Mingle is a nerdddddddddddd
		SmartDashboard.putNumber("True Gyro Heading", getHeading());
	}
	
	/**
	* Controls the left and right sides of the drive directly with voltages.
	*
	* @param leftVolts  the commanded left output
	* @param rightVolts the commanded right output
	*/
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		double batteryVolt = RobotController.getBatteryVoltage();
		drive(ControlMode.PercentOutput, leftVolts / batteryVolt, rightVolts / batteryVolt, 0);
	}
	
	/**
	* Resets the drive encoders to currently read a position of 0.
	*/
	public void resetEncoders() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);
	}
	
	/**
	* Gets the average distance of the two encoders.
	*
	* @return the average of the two encoder readings
	*/
	public double getAverageEncoderDistance() {
		double[] distance = getDtDistance();
		return (distance[0] + distance[1]) / 2.0;
	}
	
	/**
	* Zeroes the heading of the robot.
	*/
	public void zeroHeading() {
		m_gyro.setFusedHeading(0);
	}
	
	/* Returns in degrees */
	public double getHeading() {
		return m_gyro.getFusedHeading();
	}
	

	public Rotation2d getGyroRotation() {
		return new Rotation2d(Math.toRadians(m_gyro.getFusedHeading()));
	}
	
	// Returns in meters
	public double[] getDtDistance() {
		double left = Units.dtRotationsToMeters(Units.dtTickstoRotations(leftMaster.getSelectedSensorPosition()));
		double right = Units.dtRotationsToMeters(Units.dtTickstoRotations(rightMaster.getSelectedSensorPosition()));
		return new double[]{left, right};
	}

	// Returns in meters/second
	public double[] getDtVelocity() {
		double left = Units.dtRotationsToMeters(Units.dtTickstoRotations(leftMaster.getSelectedSensorVelocity() * 10));
		double right = Units.dtRotationsToMeters(Units.dtTickstoRotations(rightMaster.getSelectedSensorVelocity() * 10));
		return new double[]{left, right};
	}
}