package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.drivetrain.TurnToAngleCommand;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnOffLimelightCommand;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.Trajectories;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.Vision;

public class SixBallAutoCommand extends SequentialCommandGroup {
    /**
     * Get balls, shoot, get more balls, shoot!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public SixBallAutoCommand(Vision vision, Drivetrain drivetrain, Feeder feeder, Conveyor conveyor, IntakeArm intakeArm, Shooter shooter, Banana banana) {
        addCommands(
            new TurnAndShootFullAutoCommand(vision, drivetrain, feeder, conveyor, intakeArm, shooter, banana),
            new ParallelDeadlineGroup(
                // new DriveFollowTrajectory(drivetrain, Trajectories.MiddleToTrench[0], Trajectories.MiddleToTrench[1], false),
                new TurnOffLimelightCommand(vision),
                new AngleIntakeCommand(intakeArm, false).withTimeout(0.69),
                new IntakeBallCommand(intakeArm, 0.69, false), // never ends
                new MoveConveyorCommand(conveyor, feeder, shooter, 0.8, false),
                new FeedShooterCommand(feeder, shooter, 0.1, true)
            ),
            new TurnAndShootFullAutoCommand(vision, drivetrain, feeder, conveyor, intakeArm, shooter, banana)
        );
    }

}