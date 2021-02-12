package frc.robot.constants;

import frc.robot.subsystems.TrajectoryGenerator;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Trajectories {
    
    /**
     * Middle, aligned center with goal: new Waypoint(3.138, -2.45, 0)
     */

    public static final Trajectory[] LeftToTrench = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(3.138, -0.718, 0),
        new Waypoint(8.04, -0.718, 0)
    });

    public static final Trajectory[] MiddleToTrench = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(3.138, -2.45, 0),
        new Waypoint(5.45, -0.718, 0),
        new Waypoint(8.04, -0.718, 0)
    });

    public static final Trajectory[] TrenchToColorWheel = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(8.04, -0.718, 0),
        new Waypoint(9.74, -0.718, 0)
    });

    public static final Trajectory[] RightToGenerator = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(3.138, -4.105, 0), // align center with second ball 
        new Waypoint(5.646, -3.942, Pathfinder.d2r(20.0))
    });

    public static final Trajectory[] RightToOppColorWheel = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(3.138, -7.5, 0), // align center with center of opp color wheel
        new Waypoint(6.3, -7.5, 0)
    });

    public static final Trajectory[] OppColorWheelToGoal = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(6.3, -7.5, 0),
        new Waypoint(3.138, -3.454, Pathfinder.d2r(-18.5))
    });
    
    public static final Trajectory[] MoveOneMeter = TrajectoryGenerator.generateTrajectoryWithWaypoints(new Waypoint[] {
        new Waypoint(0, 0, 0),
        new Waypoint(1, 0, 0)
    });
}