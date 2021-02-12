package frc.robot.commandgroups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.Util;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.FeedShooterIfHasBallCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class ShootBallCommand extends SequentialCommandGroup {
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public ShootBallCommand(Feeder feeder, Conveyor conveyor, IntakeArm intakeArm, Shooter shooter, boolean inverted) {
		super(
			new PrintCommand("Shoot Ball"),
			new ParallelDeadlineGroup(
				new ParallelCommandGroup(
					new WaitUntilCommand(() -> shooter.atSpeed(RobotConstants.FLYWHEEL_RPM_DEADBAND)),
					new WaitUntilCommand(() -> feeder.hasBall())
				),
				new MoveConveyorCommand(conveyor, feeder, shooter, 0.6, inverted),
				new IntakeBallCommand(intakeArm, 0.5, inverted)
			),
			new ParallelDeadlineGroup(
				// new FeedShooterCommand(feeder, shooter, inverted).withTimeout(0.08),
				new FeedShooterIfHasBallCommand(feeder, shooter, inverted, true),
				// new MoveConveyorCommand(conveyor, feeder, shooter, 0.2, inverted),
				new IntakeBallCommand(intakeArm, 0.5, inverted)
			)
		);
	}
}