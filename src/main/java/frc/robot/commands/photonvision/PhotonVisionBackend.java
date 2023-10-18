package frc.robot.commands.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.VisionBackend;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

public class PhotonVisionBackend extends VisionBackend {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionBackend(String name, Transform3d camera_transform) throws IOException {
        camera = new PhotonCamera(name);
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
//        File aprilTagLayoutFile = new File(Filesystem.getDeployDirectory(), "AprilTagLayout/Template");


        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, camera_transform);
    }

    // DISTANCE_FILTER: This variable is used to filter out measurements that are too far away.
// If the distance to a target is greater than this value, the measurement is rejected.
// This can help improve the accuracy of your vision system by ignoring targets that are too far to be measured accurately.

// AMBIGUITY_FILTER: This variable is used to filter out measurements that have too much ambiguity.
// If the pose ambiguity of a target is greater than this value, the measurement is rejected.
// Pose ambiguity can occur when multiple targets are detected and it's unclear which target corresponds to which real-world object.
// By filtering out high-ambiguity measurements, you can improve the reliability of your vision system.

// PHOTON_VISION_STD_DEV: This variable represents the standard deviation of the measurements from PhotonVision.
// It's used in the forMeasurement() method to calculate a Vector<N3> representing the standard deviation for a given distance and count.
// This can be useful for estimating the uncertainty or noise in your measurements.


    @Override
    public Optional<Measurement> getMeasurement() {
        return poseEstimator.update().flatMap((result) -> {
            RobotContainer.field.getObject("Vision Measurement " + camera.getName()).setPose(result.estimatedPose.toPose2d());

            if (result.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > Constants.VisionConstants.DISTANCE_FILTER || result.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.AMBIGUITY_FILTER) {
                return Optional.empty();
            }

            // Reject pose estimates outside the field
            if (result.estimatedPose.toPose2d().getX() < 0 || result.estimatedPose.toPose2d().getX() > Constants.FieldConstants.fieldLength ||
                    result.estimatedPose.toPose2d().getY() < 0 || result.estimatedPose.toPose2d().getY() > Constants.FieldConstants.fieldWidth) {
                return Optional.empty();
            }

            return Optional.of(new Measurement(
                    result.timestampSeconds,
                    result.estimatedPose,
                    Constants.VisionConstants.PHOTON_VISION_STD_DEV.forMeasurement(result.targetsUsed.get(0).getBestCameraToTarget().getX(), result.targetsUsed.size())
            ));
        });
    }

    public interface StandardDeviation {
        Vector<N3> forMeasurement(double distance, int count);
    }
}