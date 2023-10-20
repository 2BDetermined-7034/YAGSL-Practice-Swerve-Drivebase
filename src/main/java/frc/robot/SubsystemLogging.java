package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Arrays;

public interface SubsystemLogging {
    
    default void log(String key, String val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, boolean val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, double val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, Pose2d... val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, SwerveModulePosition[] val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), Arrays.toString((val)));
    }
    default void log(String key, SwerveModuleState[] val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, ChassisSpeeds val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), String.valueOf((val)));
    }
    default void log(String key, float val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, Translation2d val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), String.valueOf((val)));
    }
    default void log(String key, Transform3d val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), String.valueOf((val)));
    }
    default void log(String key, Rotation2d val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), String.valueOf((val)));
    }
    default void log(String key, PhotonPipelineResult val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val).hasTargets());
    }


}
