package frc.robot.commands.photonvision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveToAprilTagPID extends Command {
    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private final PIDController distanceController = new PIDController(0.1, 0.0, 0.0);
    SwerveSubsystem swerveDrive;

    public DriveToAprilTagPID(SwerveSubsystem swerveSubsystem) {
        this.swerveDrive = swerveSubsystem;
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            var pose = result.getBestTarget().getBestCameraToTarget();
            var translation = new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY());
            double currentDistance = translation.getNorm();
            double desiredDistance = 2.0;
            double speed = distanceController.calculate(currentDistance, desiredDistance);
            double yaw = result.getBestTarget().getYaw();
            swerveDrive.drive(translation.rotateBy(new Rotation2d(speed)), yaw, false, false);
        }


    }
}
