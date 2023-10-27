// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Limelight.LimelightAlign;
import frc.robot.commands.drive.AbsoluteFieldDrive;
import frc.robot.commands.drive.ControllerDrive;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.photonvision.TargetAiming;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final PS4Controller driverController = new PS4Controller(Constants.ControllerConstants.ps5Controller);
  public static final XboxController logiController = new XboxController(Constants.ControllerConstants.logitechController);
  public static final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
  public static final LimeLight limelight = new LimeLight();
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  public static Field2d field = new Field2d();

  public static final TeleopDrive teleopDrive = new TeleopDrive(drivebase,
          driverController::getLeftX, driverController::getLeftY,
          () -> driverController.getRawAxis(2),
          () -> true, false, true
          );

  AbsoluteFieldDrive closedAbsoluteDrive = new AbsoluteFieldDrive(drivebase,

          () -> MathUtil.applyDeadband(driverController.getLeftY(),
                  0.1),
          () -> MathUtil.applyDeadband(driverController.getLeftX(),
                  0.1),
          () -> -driverController.getRawAxis(2),
          false);

  private static final ControllerDrive controlDrive = new ControllerDrive(drivebase, () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1), () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.1), () -> MathUtil.applyDeadband(driverController.getRawAxis(2) / 1.5, 0.1), true);
  private static final ControllerDrive controlDriveLogi = new ControllerDrive(drivebase, () -> MathUtil.applyDeadband(logiController.getLeftX(), 0.1), () -> MathUtil.applyDeadband(logiController.getLeftY(), 0.1), () -> MathUtil.applyDeadband(logiController.getRawAxis(4) / 1.5, 0.1), true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws IOException {
    chooser.addOption("Straight Auto", AutoFactory.runTestAutoForwardOnly(drivebase));
    chooser.addOption("Test Curved Auto", AutoFactory.testCurvedAuto(drivebase));
    chooser.setDefaultOption("Do nothing", new WaitCommand(1));

    SmartDashboard.putData("Auto",chooser);



    // drivebase.setDefaultCommand(controlDriveLogi);
    drivebase.setDefaultCommand(controlDrive);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

//    new Trigger(driverController::getShareButton).onTrue(drivebase.runOnce(drivebase::zeroGyro));
//    new Trigger(driverController::getCircleButton).whileTrue(drivebase.run(drivebase::goToZero));
//    new Trigger(driverController::getCrossButton).onTrue(drivebase.runOnce(drivebase::lock));
//
//    new Trigger(logiController::getBackButton).onTrue(drivebase.runOnce(drivebase::zeroGyro));
//    new Trigger(logiController::getAButton).onTrue(drivebase.runOnce(drivebase::lock));

    new Trigger(driverController::getCircleButton).toggleOnTrue(new LimelightAlign(drivebase, limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
//     return AutoFactory.runTestAutoForwardOnly(drivebase);
//    return new ControllerDrive(drivebase, () -> 0, () -> 0, () -> 0, true);
    return chooser.getSelected();
  }
}
