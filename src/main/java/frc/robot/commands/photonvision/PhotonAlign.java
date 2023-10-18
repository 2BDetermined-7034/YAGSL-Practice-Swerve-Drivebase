package frc.robot.commands.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonAlign extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonCamera camera;
    private final int tagId;
    private final double tolerance;

    public PhotonAlign(SwerveSubsystem swerveSubsystem, PhotonCamera camera, int tagId, double tolerance) {
        this.swerveSubsystem = swerveSubsystem;
        this.camera = camera;
        this.tagId = tagId;
        this.tolerance = tolerance;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Set up PhotonVision pipeline
        camera.setPipelineIndex(0);
    }

    @Override
    public void execute() {
        // Get list of tracked targets
        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();

        // Find target with desired ID
        PhotonTrackedTarget target = null;
        for (PhotonTrackedTarget t : targets) {
            if (t.getFiducialId() == tagId) {
                target = t;
                break;
            }
        }
        if (target == null) {
            return;
        }

        // Get AprilTag pose in camera frame
        Pose2d tagPose = target.getPose();

        // Get robot pose
        Pose2d robotPose = swerveSubsystem.getPose();

        // Calculate field-relative pose
        Pose2d fieldRelativePose = robotPose.plus(tagPose);

        // Align robot to AprilTag with drive method
        Translation2d translation = fieldRelativePose.getTranslation();
        double rotation = fieldRelativePose.getRotation().getRadians();
        swerveSubsystem.drive(translation, rotation, true, true);

        // Check if we are within tolerance
        if (translation.getNorm() < tolerance) {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
