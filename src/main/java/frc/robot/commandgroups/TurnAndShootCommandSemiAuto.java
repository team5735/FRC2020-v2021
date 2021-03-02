package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.FeedShooterIfHasBallCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnOffLimelightCommand;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TurnAndShootCommandSemiAuto extends SequentialCommandGroup {

    private Vision vision;
    private Drivetrain drivetrain;
    private Feeder feeder;
    private Conveyor conveyor;
    private IntakeArm intakeArm;
    private Shooter shooter;
    private Banana banana;

    /**
     * Turn to target, ramp up shooter, feed the shooter IF there is a ball ready
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public TurnAndShootCommandSemiAuto(Vision vision, Drivetrain drivetrain, Feeder feeder, Conveyor conveyor, IntakeArm intakeArm, Shooter shooter, Banana banana) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.conveyor = conveyor;
        this.intakeArm = intakeArm;
        this.shooter = shooter;
        this.banana = banana;

        TurnToTargetCommand turnToTargetCommand = new TurnToTargetCommand(vision, drivetrain);
        
        addCommands(
            // https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html
            new ParallelCommandGroup(
                turnToTargetCommand,
                new SequentialCommandGroup(
                    new RampShooterCommand(shooter, vision, banana, feeder, 3600),
                    new WaitUntilCommand(turnToTargetCommand),
                    new RampShooterCommand(shooter, vision, banana, feeder, true),
                    new ShootBallCommand(feeder, conveyor, intakeArm, shooter, false),
                    new FeedShooterIfHasBallCommand(feeder, shooter, false, false)
                )
            )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        // drivetrain.setDriveMode(DriveMode.DISABLED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // drivetrain.setDriveMode(DriveMode.STATIC_DRIVE);
    }

}