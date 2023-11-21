package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;


public class LimelightDrive extends CommandBase implements SubsystemLogging {
    private final LimeLight limeLight;
    private final SwerveSubsystem swerve;
    private PIDController pid;
    boolean opposite;
    DoubleSupplier x,y,rot;
    public LimelightDrive(SwerveSubsystem swerve, LimeLight limeLight, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        this.x = x;
        this.y = y;
        this.swerve = swerve;
        this.limeLight = limeLight;
        this.rot = rot;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double output = 0;

        if(limeLight.isTargetAvailable()) {

            double tx = limeLight.getTargetOffsetX();
            pid = new PIDController(0.01, 0, 0);
            pid.setTolerance(5);
            pid.setSetpoint(0);
            double currentAngle = swerve.getAngle().getDegrees();

            output = -pid.calculate(tx);

            double targetVert = limeLight.getVert();
            double targetHor = limeLight.getHor();
            double aspectRatio = limeLight.calculateAspectRatio(targetHor, targetVert);

            opposite = oppositeTeam(aspectRatio, false);


            log("PID output", output);
            log("Current Angle", currentAngle);
            log("TX", tx);
            log("At Setpoint", pid.atSetpoint());
            log("Horizontal", targetHor);
            log("Verticle", targetVert);
            log("Target", opposite);
            log("Aspect Ratio", aspectRatio);


            if (opposite) {


            }

        }

        if(opposite && limeLight.isTargetAvailable()) {
            swerve.drive(new Translation2d(y.getAsDouble(), x.getAsDouble()), output, true, true);

        } else {
            swerve.drive(new Translation2d(y.getAsDouble(), x.getAsDouble()), rot.getAsDouble(), true, true);

        }


        }

//    public static void noTarget(SwerveSubsystem swerve) {
//        swerve.drive(new Translation2d(0,0), 0.3, false, true);
//    }



    private static boolean oppositeTeam(double aspectRatio, boolean override) {
        boolean target = false;

        if(override) {
            target = true;
            return target;
        }

        switch(DriverStation.getAlliance()) {
            case Red:
                // target blue

                target = aspectRatio > 3.5;
                break;

            case Blue:
                // target red

                target = aspectRatio < 3.4;
                break;

        }
        return target;
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
