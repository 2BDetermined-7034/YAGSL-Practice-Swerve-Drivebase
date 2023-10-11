package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.PathFactory;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoFactory {

    // TODO Tune PID as anything under 3 meters overshoots
    public static Command runTestAutoForwardOnly(SwerveSubsystem drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Test", new PathConstraints(1, 1));

        return new PathFactory(drive, path, true).getCommand();
    }



}