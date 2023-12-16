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

    /**
     * Command for making the drivebase track a retroreflective tape target with rotation while maintaining position from the controller
     * @param swerve drivebase
     * @param limeLight limelight
     * @param x X joystick input
     * @param y Y joystick input
     * @param rot theta joystick input
     */
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

        log("Target available", limeLight.isTargetAvailable());


        if(limeLight.isTargetAvailable()) {

            double tx = limeLight.getTargetOffsetX();
            pid = new PIDController(0.02, 000.00, 0.00000);
            pid.setTolerance(5);

            //Second term is for keeping the robot ahead of the target to account for shooter delay
            pid.setSetpoint(000.000 + swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.001);
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

//                target = aspectRatio > 3.5;
                target = true;
                break;

            case Blue:
                // target red

//                target = aspectRatio < 3.4;
                target = true;
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
