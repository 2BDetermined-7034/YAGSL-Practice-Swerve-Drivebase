package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class PoseEstimator {
    private final boolean usePhoton;
    private final PhotonCamera photonCamera;
    private final NetworkTable limelightTable;
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(2);

    public PoseEstimator(boolean usePhoton) {
        this.usePhoton = usePhoton;
        if (usePhoton) {
            photonCamera = new PhotonCamera("HD_Webcam_C");
            limelightTable = null;
        } else {
            photonCamera = null;
            limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        }
    }

    public void periodic() {
        poseHistory.addSample(Timer.getFPGATimestamp(), estimatePoseWithCamera());
    }

    public Pose2d estimatePoseWithCamera() {
        if (usePhoton) {
            var result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();

                /**
                 * Distance to meters is calculated using the formula:
                 * @param cameraHeightMeters The height of the camera lens in meters.
                 * @param targetHeightMeters The height of the target in meters.
                 * @param cameraPitchRadians The pitch of the camera in radians.
                 */
                double distance = PhotonUtils.calculateDistanceToTargetMeters(
                        0,
                        0,
                        0,
                        Units.degreesToRadians(target.getPitch())
                );

                Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw());

                Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distance, yaw);

                return new Pose2d(translation, new Rotation2d());
            } else {
                return new Pose2d();
            }
        } else {
            double tx = limelightTable.getEntry("tx").getDouble(0.0);
            double ty = limelightTable.getEntry("ty").getDouble(0.0);
            double ta = limelightTable.getEntry("ta").getDouble(0.0);

            return new Pose2d(new Translation2d(tx, ty), Rotation2d.fromDegrees(ta));
        }
    }
    public Translation2d estimateVelocity() {
        Pose2d currentPose = estimatePoseWithCamera();
        double currentTimestamp = Timer.getFPGATimestamp();

        if (poseHistory.getInternalBuffer().size() < 2) {
            return new Translation2d();
        }

        Optional<Pose2d> lastPoseOpt = poseHistory.getSample(currentTimestamp - 1.0);
        if (lastPoseOpt.isPresent()) {
            Pose2d lastPose = lastPoseOpt.get();

            Translation2d deltaPosition = currentPose.getTranslation().minus(lastPose.getTranslation());

            return new Translation2d(deltaPosition.getX(), deltaPosition.getY());
        } else {
            return new Translation2d();
        }
    }


}
