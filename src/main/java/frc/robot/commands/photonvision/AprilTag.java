package frc.robot.commands.photonvision;

import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTag extends CommandBase {
    // Change this to match the name of your camera
    private final PhotonCamera camera = new PhotonCamera("photonvision");

    public AprilTag() {

    }
    @Override
    public void execute() {
        // Get the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // If we have targets, get the pose of the best target
            var pose = result.getBestTarget().getBestCameraToTarget();

            // Convert the pose's translation to a Translation2d object
            var translation = new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY());

            System.out.println("Best Target Translation: " + translation);
        }
    }
}
