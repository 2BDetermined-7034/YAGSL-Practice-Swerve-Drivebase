package frc.robot.commands.photonvision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTag extends CommandBase implements SubsystemLogging {
    // Change this to match the name of your camera
    PhotonVisionSubsystem sybs;

    public AprilTag(PhotonVisionSubsystem sybs) {
        this.sybs = sybs;

        addRequirements(sybs);
    }
    @Override
    public void execute() {
        // Get the latest result from PhotonVision
        var result = sybs.getLatestResult();
        log("Has Results", result.hasTargets());
        if (result.hasTargets()) {
            // If we have targets, get the pose of the best target
            var pose = result.getBestTarget().getBestCameraToTarget();

            // Convert the pose's translation to a Translation2d object
            var translation = new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY());

            log("Pose X", pose.getTranslation().getX());
            log("Pose Y", pose.getTranslation().getY());

        }
    }
}
