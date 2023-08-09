package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.util.function.DoubleSupplier;

public class ControllerDrive extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rawAxis;

    public ControllerDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rawAxis) {
        this.swerveSubsystem = swerveSubsystem;
        this.x = x;
        this.y = y;
        this.rawAxis = rawAxis;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(x.getAsDouble(), y.getAsDouble()), rawAxis.getAsDouble(), false, true);
    }
}
