/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.RobotConstants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryGenerator {

    public TrajectoryGenerator() {
    }

    public static Trajectory[] generateTrajectoryWithWaypoints(Waypoint[] points) {
        Trajectory leftTrajectory;
        Trajectory rightTrajectory;

        long timenow = System.currentTimeMillis();
        System.out.println("@@@@@@ GENERATING TRAJECTORIES @@@@@@ " + timenow);

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.02, /*0.69 * */RobotConstants.MAX_VELOCITY_DT, 0.8, 8);
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVETRAIN_TRACK_WIDTH);
        leftTrajectory = modifier.getLeftTrajectory();
        rightTrajectory = modifier.getRightTrajectory();

        System.out.println(System.currentTimeMillis() - timenow);
        return new Trajectory[] {leftTrajectory, rightTrajectory};
    }

    public static Trajectory[] readTrajectories(String fileName) {

        File leftTraj = new File(Filesystem.getOperatingDirectory() + "/trajectories/" + fileName + "_left.csv");
        File rightTraj = new File(Filesystem.getOperatingDirectory() + "/trajectories/" + fileName + "_right.csv");
        System.out.println("LEFT TRAJ: " + (leftTraj.exists()) + ", RIGHT TRAJ: " + (rightTraj.exists()));

        try {
            Trajectory left = Pathfinder.readFromCSV(leftTraj);
            Trajectory right = Pathfinder.readFromCSV(rightTraj);
            return new Trajectory[] {left, right};
        } catch(IOException exception) {
            exception.printStackTrace();
        }

        return null;
    }

    public static void exportTrajectories(Trajectory[] trajectories, String fileName) {
        File left = new File(Filesystem.getOperatingDirectory() + "/trajectories/" + fileName + "_left.csv");
        File right = new File(Filesystem.getOperatingDirectory() + "/trajectories/" + fileName + "_right.csv");
        try {
            if (left.createNewFile() || right.createNewFile()) {
                
            }

            Pathfinder.writeToCSV(left, trajectories[0]);
            Pathfinder.writeToCSV(right, trajectories[1]);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
