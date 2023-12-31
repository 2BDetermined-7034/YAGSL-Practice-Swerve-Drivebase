package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class PathFactory {
    private final Command followTrajectoryCommand;
    SwerveSubsystem m_swerveDrive;

    public PathFactory(SwerveSubsystem drive, PathPlannerTrajectory path, boolean isFirstPath) {

        m_swerveDrive = drive;
        m_swerveDrive.addTrajectory(path);


        followTrajectoryCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        m_swerveDrive.setPosition(path.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        path,
                        m_swerveDrive::getPosition, // Pose supplier
                        m_swerveDrive.getKinematics(), // SwerveDriveKinematics
                        new PIDController(0.01, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(0.01, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0.01, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        m_swerveDrive::setModuleStates, // Module states consumer
                        m_swerveDrive // Requires this drive subsystem
                )
        );
    }

//    public static PathPlannerTrajectory pathMaker(List<PathPoint> points) {
//        return PathPlanner.generatePath(
//                new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration),
//                points
//        );
//    }
//
//    public static PathPlannerTrajectory createTagPath(Pose2d startPos, Pose2d endPos) {
//
//        return PathPlanner.generatePath(
//                new PathConstraints(1, 1),
//                pose2dToPathpoint(startPos),
//                pose2dToPathpoint(endPos)
//
//        );
//
//
//        //TrajectoryGenerator.generateTrajectory(startPos, endPos, new);
//
//
//    }

    public static PathPoint pose2dToPathpoint(Pose2d point) {
        return new PathPoint(point.getTranslation(), point.getRotation());
    }

    public Command getCommand() {
        return followTrajectoryCommand.andThen(m_swerveDrive::stop);
    }


}
