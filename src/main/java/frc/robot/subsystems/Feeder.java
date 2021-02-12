/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

	private final TalonSRX feeder;
	private final DigitalInput beambreak;

	/**
	 * Creates a new Feeder.
	 */
	public Feeder() {
		feeder = Drivetrain.gyroHost; // shared TalonSRX
		beambreak = new DigitalInput(9);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// System.out.println(hasBall());
	}

	/**
	 * @return Position, in sensor units
	 */
	public void feedShooter(double speed, boolean inverted) {
		feeder.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * speed);
	}

	public boolean hasBall() {
		if(beambreak.get() == false) System.out.println("################ BALL INSIDE FEEDER | BEAM BREAK");
		return !beambreak.get();
	}
}
