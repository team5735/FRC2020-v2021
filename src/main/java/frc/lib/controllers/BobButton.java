package frc.lib.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.controllers.BobXboxController.XboxButton;

public class BobButton extends JoystickButton {

	public BobButton(GenericHID joystick, int buttonNumber) {
		super(joystick, buttonNumber);
	}

	public BobButton(XboxController joystick, XboxButton button) {
		super(joystick, button.value);
	}

	public BobButton(BobXboxController joystick, XboxButton button) {
		super(joystick, button.value);
	}

}
