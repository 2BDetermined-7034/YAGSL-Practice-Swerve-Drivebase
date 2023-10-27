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
 public static class Vision {
    public static final double limeLightMountAngleDegrees = 0;
    public static final double goalHeighInches = 53.75;
    public static final double limeligtLensHeighInches = 0;
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
