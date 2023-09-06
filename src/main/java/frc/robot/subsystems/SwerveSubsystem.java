package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveController;
import swervelib.imu.NavXSwerve;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.telemetry.SwerveDriveTelemetry;


public class SwerveSubsystem extends SubsystemBase {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve/neo");
    SwerveDrive swerveDrive  = new SwerveParser(swerveJsonDirectory).createSwerveDrive();


    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this SwerveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static SwerveSubsystem INSTANCE;

    static {
        try {
            INSTANCE = new SwerveSubsystem();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Returns the Singleton instance of this SwerveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SwerveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this SwerveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public SwerveSubsystem() throws IOException {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fieldx", swerveDrive.getFieldVelocity().vxMetersPerSecond);
        SmartDashboard.putNumber("Fieldy", swerveDrive.getFieldVelocity().vyMetersPerSecond);

        SmartDashboard.putNumber("Fieldo", swerveDrive.getFieldVelocity().omegaRadiansPerSecond);

        swerveDrive.updateOdometry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }
    public SwerveController getSwerveController() {
        return swerveDrive.getSwerveController();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    public void goToZero(){
        double yaw = swerveDrive.getYaw().getRadians();
        SwerveController sc = swerveDrive.getSwerveController();
        sc.getTargetSpeeds(0, 0, 0, yaw);
        swerveDrive.drive(new Translation2d(), sc.getTargetSpeeds(0, 0, 0, yaw).omegaRadiansPerSecond * 0.1, true, true);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }



}

