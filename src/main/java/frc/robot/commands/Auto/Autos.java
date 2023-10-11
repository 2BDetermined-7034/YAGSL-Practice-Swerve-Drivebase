// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.HashMap;
import java.util.List;

public final class Autos
{

  /**
   * April Tag field layout.
   */
  private static AprilTagFieldLayout aprilTagField = null;

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command driveAndSpin(SwerveSubsystem swerve)
  {
    return Commands.sequence(
        new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true, true), swerve)));
  }

  public static Command move1Meter(SwerveSubsystem swerve) {
    boolean               onTheFly = false; // Use the path defined in code or loaded from PathPlanner.
    PathPlannerTrajectory example;
    if (onTheFly)
    {
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      example = PathPlanner.generatePath(
          new PathConstraints(4, 3),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
// position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
// position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(5, 5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
          // position, heading(direction of travel), holonomic rotation
                                        );
    } else
    {
      List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("Test", new PathConstraints(4, 3));
      // This is just an example event map. It would be better to have a constant, global event map
      // in your code that will be used by all path following commands.
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));

      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          swerve::getPose,
          swerve::resetOdometry,
          new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
          new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
          swerve::setChassisSpeeds,
          eventMap,
          false,
          swerve
      );
      return Commands.sequence(autoBuilder.fullAuto(example1));
    }
    return Commands.sequence(new FollowTrajectory(swerve, example, true));

  }


}