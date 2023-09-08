package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoPathFactory {

    public static Command testAuto(SwerveSubsystem drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Test", new PathConstraints(5, 3));

        return new PathFactory(drive, path, true).getCommand();
    }
}
