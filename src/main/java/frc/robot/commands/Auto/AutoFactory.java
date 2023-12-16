package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.PathFactory;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoFactory {

    // TODO Tune PID as anything under 3 meters overshoots
    public Command straightPath(SwerveSubsystem drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Straight", new PathConstraints(1, 1));

        return new PathFactory(drive, path, true).getCommand();
    }

    public static Command testCurvedAuto(SwerveSubsystem drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Curved Path", new PathConstraints(1,1));

        return new PathFactory(drive, path, true).getCommand();
    }



}