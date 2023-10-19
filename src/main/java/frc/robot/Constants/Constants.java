// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.photonvision.PhotonVisionBackend;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int ps5Controller = 0;
    public static final int logitechController = 1;
  }
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static class VisionConstants {
    // FIXME: actually measure these constants

    public static final Transform3d BACK_CAM_TRANSFORM = new Transform3d(
            new Translation3d(-0.132869, 0.060930, -0.463550),
            new Rotation3d(0, 0, Math.PI)
    );

    public static final Transform3d LEFT_CAM_TRANSFORM = new Transform3d(
            new Translation3d(.218, 0.231, -0.25),
            new Rotation3d(0, Units.degreesToRadians(15), 0)
    );

    public static final PhotonVisionBackend.StandardDeviation PHOTON_VISION_STD_DEV =
            (distance, count) -> {
              double distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2);
              double translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.05;
              double rotationalStdDev = 0.2 * distanceMultiplier + 0.1;
              return VecBuilder.fill(
                      translationalStdDev,
                      translationalStdDev,
                      rotationalStdDev
              );
            };

    public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double AMBIGUITY_FILTER = 0.3;
    public static final double DISTANCE_FILTER = FieldConstants.fieldLength / 2;
  }

  public static class FieldConstants {
    //FIXME Change the field length and width to match the actual field
    public static final double fieldLength = 16.542;
    public static final double fieldWidth = 8.0137;
  }
  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.1, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.1, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.1, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }
}
