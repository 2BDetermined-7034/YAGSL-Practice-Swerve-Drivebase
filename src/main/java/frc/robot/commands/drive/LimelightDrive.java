package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;


public class LimelightDrive extends CommandBase implements SubsystemLogging {
    private final LimeLight limeLight;
    private final SwerveSubsystem swerve;
    private PIDController pid;
    DoubleSupplier x;
    DoubleSupplier y;

    public LimelightDrive(SwerveSubsystem swerve, LimeLight limeLight, DoubleSupplier x, DoubleSupplier y) {
        this.x = x;
        this.y = y;
        this.swerve = swerve;
        this.limeLight = limeLight;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(!limeLight.isTargetAvailable()) {
            swerve.drive(new Translation2d(0,0), 0.4, false, true);
        } else {

            double tx = limeLight.getTargetOffsetX();
            pid = new PIDController(0.01, 0, 0);
            pid.setTolerance(0);
            pid.setSetpoint(0);
            double currentAngle = swerve.getAngle().getDegrees();


            double output = -pid.calculate(tx);


            log("PID output", output);
            log("Current Angle", currentAngle);
            log("TX", tx);
            log("At Setpoint", pid.atSetpoint());


            swerve.drive(new Translation2d(y.getAsDouble(),x.getAsDouble()), output, true, true);
        }

    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
