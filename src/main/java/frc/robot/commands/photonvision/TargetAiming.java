package frc.robot.commands.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.time.format.DecimalStyle;
import java.util.Arrays;


public class TargetAiming extends CommandBase implements SubsystemLogging {
    private final SwerveSubsystem swerveSubsystem;
    private PhotonCamera camera;
    ShuffleboardTab photonVisionTab = Shuffleboard.getTab("PhotonVision");
    GenericEntry targetFound, desiredID, x, y;



    public TargetAiming(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        camera = new PhotonCamera("HD_Webcam_C615");
        targetFound = Shuffleboard.getTab("PhotonVision").add("Target Found", false).getEntry();
        desiredID = Shuffleboard.getTab("PhotonVision").add("Desired ID?", false).getEntry();
        x = Shuffleboard.getTab("PhotonVision").add("X", 0.0).getEntry();
        y = Shuffleboard.getTab("PhotonVision").add("Y", 0.0).getEntry();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            targetFound.setBoolean(result.hasTargets());
            // Define your array of desired AprilTag IDs
            int[] desiredAprilTagIds = {1,2,3,4};

            // Check if the detected AprilTag's ID is in your array
            int detectedAprilTagId = result.getBestTarget().getFiducialId();

            boolean idIsDesired = Arrays.stream(desiredAprilTagIds).anyMatch(id -> id == detectedAprilTagId);

            if (idIsDesired) {
                desiredID.setBoolean(true);

                Transform3d transform3D = result.getBestTarget().getBestCameraToTarget();

                Translation2d translation = new Translation2d(transform3D.getTranslation().getX(), transform3D.getTranslation().getY());

                // Calculate the distance to the target
                double distanceToTarget = Math.hypot(translation.getX(), translation.getY());

                // Only drive towards the target if it's more than 0.5 meters away
                if (distanceToTarget > 0.5) {
                    Rotation2d rotation = new Rotation2d(transform3D.getRotation().toRotation2d().getRadians());

                    Pose2d pose = new Pose2d(translation, rotation);

                    log("PhotonVision pose", pose);
                    log("PhotonVision translation", translation);
                    log("PhotonVision rotation", rotation);
                    log("PhotonVision transform3D", transform3D);
                    log("PhotonVision result", result);
                    log("PhotonVision X", translation.getX());
                    log("PhotonVision Y", translation.getY());

                    targetFound.setBoolean(true);

                    x.setDouble(translation.getX());
                    y.setDouble(translation.getY());



//                    swerveSubsystem.drive(translation, rotation.getRadians(), false, false);
                } else {
                    desiredID.setBoolean(false);
                }
            }
        } else {
            targetFound.setBoolean(false);
        }
    }


    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
