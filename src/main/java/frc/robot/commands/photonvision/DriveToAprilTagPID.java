package frc.robot.commands.photonvision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.SubsystemLogging;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveToAprilTagPID extends CommandBase implements SubsystemLogging {
    PhotonVisionSubsystem photonVisionSubsystem;
    private final PIDController distanceController = new PIDController(0.1, 0.0, 0.0);
    private final PIDController angleController = new PIDController(0.1, 0.0, 0.0);
    SwerveSubsystem swerveDrive;

    public DriveToAprilTagPID(SwerveSubsystem swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
        this.swerveDrive = swerveSubsystem;
        this.photonVisionSubsystem = photonVisionSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        var result = photonVisionSubsystem.getLatestResult();

        log("Has Result", result.hasTargets());

        if (!result.hasTargets()) {
            swerveDrive.drive(new Translation2d(0,0), 0.5, true, true);
        } else {
            var pose = result.getBestTarget().getBestCameraToTarget();

            var translation = new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY());

            double currentDistance = translation.getNorm();
            double desiredDistance = 1.0;
            //angleController.setTolerance(6);

            double xSpeed = distanceController.calculate(pose.getX(), desiredDistance);
            double ySpeed = distanceController.calculate(pose.getY(), 0);

            double rotSpeed = angleController.calculate(Math.toDegrees(pose.getRotation().getAngle()), 180);

            double yaw = result.getBestTarget().getYaw();

            //swerveDrive.drive(translation.rotateBy(new Rotation2d(speed)), yaw, true, false);
            swerveDrive.drive(new Translation2d(-xSpeed, -ySpeed), -0.1 * rotSpeed, false, true);

            log("Y Speed", ySpeed);
            log("X Speed", xSpeed);
            log("Rotation Speed", rotSpeed);

        }


    }


}
