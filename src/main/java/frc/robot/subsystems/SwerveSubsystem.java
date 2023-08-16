package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveController;
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
        swerveDrive.updateOdometry();
//        swerveDrive.drive(new Translation2d(1,1), 0, false, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public SwerveController getSwerveController() {
        return swerveDrive.getSwerveController();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }


}

