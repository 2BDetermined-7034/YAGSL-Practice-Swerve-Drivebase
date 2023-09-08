package frc.robot.commands;

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
import swervelib.SwerveController;
import swervelib.SwerveDrive;

import java.util.List;

public class PathFactory {
    private final Command followTrajectoryCommand;
    SwerveSubsystem m_swerveDrive;
    PPSwerveControllerCommand swerveControllerCommand;
    static SwerveController controller;

    public PathFactory(SwerveSubsystem drive, PathPlannerTrajectory path, boolean isFirstPath) {
        controller = SwerveSubsystem.getInstance().getSwerveController();
        m_swerveDrive = drive;
        m_swerveDrive.addTrajectory(path);

        followTrajectoryCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        m_swerveDrive.setPosition(path.getInitialHolonomicPose());
                    }
                })
        );

        new PPSwerveControllerCommand(
        path, 
        m_swerveDrive::getPosition, 
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        null, 
        null);

        // new PPSwerveControllerCommand(
        //                 path,
        //                 m_swerveDrive::getPosition, // Pose supplier
        //                 m_swerveDrive.getKinematics(), // SwerveDriveKinematics
        //                 new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //                 new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // Y controller (usually the same values as X controller)
        //                 new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //                 m_swerveDrive::setModuleStates, // Module states consumer
        //                 m_swerveDrive // Requires this drive subsystem
        //         )

    }

    public static PathPlannerTrajectory pathMaker(List<PathPoint> points) {

        return PathPlanner.generatePath(
                new PathConstraints(1, 2),
                points
        );
    }

    public static PathPlannerTrajectory createTagPath(Pose2d startPos, Pose2d endPos) {

        return PathPlanner.generatePath(
                new PathConstraints(1, 1),
                pose2dToPathpoint(startPos),
                pose2dToPathpoint(endPos)

        );


        //TrajectoryGenerator.generateTrajectory(startPos, endPos, new);


    }

    public static PathPoint pose2dToPathpoint(Pose2d point) {
        return new PathPoint(point.getTranslation(), point.getRotation());
    }

    public Command getCommand() {
        return followTrajectoryCommand.andThen(m_swerveDrive::stop);
    }


}