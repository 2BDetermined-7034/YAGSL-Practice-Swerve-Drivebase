package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

import java.util.function.DoubleSupplier;

public class ControllerDrive extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rawAxis;
    private final boolean isOpenLoop;
    private final SwerveController controller;

    public ControllerDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rawAxis, boolean isOpenLoop) {
        this.swerveSubsystem = swerveSubsystem;
        this.x = x;
        this.y = y;
        this.rawAxis = rawAxis;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerveSubsystem.getSwerveController();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(y.getAsDouble(), x.getAsDouble()), rawAxis.getAsDouble() , true, isOpenLoop);
        // REMEMBER - X (first variable) in the translation 2D corresponds to forwards movement!
        // "positive x is torwards the bow (front) and positive y is torwards port (left)" WHYYY
        // 💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀
    }


}
