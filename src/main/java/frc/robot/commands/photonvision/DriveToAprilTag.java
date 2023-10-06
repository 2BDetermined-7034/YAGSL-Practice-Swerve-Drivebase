package frc.robot.commands.photonvision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToAprilTag extends CommandBase {
    // Change this to match the name of the camera
    private final PhotonCamera camera = new PhotonCamera("photonvision");

    SwerveSubsystem swerveDrive;

    public DriveToAprilTag(SwerveSubsystem swerveSubsystem) {
        this.swerveDrive = swerveSubsystem;
    }

    @Override
    public void execute() {
        // Get the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Get the pose of the best target
            var pose = result.getBestTarget().getBestCameraToTarget();

            // Convert the pose's translation to a Translation2d object
            var translation = new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY());

            // Calculate the current distance to the target
            double currentDistance = translation.getNorm();

            // Calculate the error in distance
            // Desired distance to maintain from the AprilTag (in meters)
            double desiredDistance = 2.0;
            double distanceError = desiredDistance - currentDistance;

            // Use a simple proportional controller to drive towards or away from the target
            double speed = 0.1 * distanceError;

            // Get the yaw of the best target
            double yaw = result.getBestTarget().getYaw();

            // Use the translation and yaw to drive and rotate the robot
            swerveDrive.drive(translation.rotateBy(new Rotation2d(speed)), yaw, false, false);
        }
    }
}
