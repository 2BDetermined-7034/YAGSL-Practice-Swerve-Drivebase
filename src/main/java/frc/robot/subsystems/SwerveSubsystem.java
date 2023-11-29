package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.SubsystemLogging;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveController;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;


public class SwerveSubsystem extends SubsystemBase implements SubsystemLogging {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve/neo");
    SwerveDrive swerveDrive  = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
    private static final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final Field2d field2d;
    private final SwerveDrivePoseEstimator estimator;


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
        // SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH; telemetry
        field2d = new Field2d();
        // 24 is the length and width
        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                new Translation2d(24 / 2.0, 24 / 2.0), // 24 is the length and width
                new Translation2d(24 / 2.0, -24 / 2.0),
                new Translation2d(-24 / 2.0, 24 / 2.0),
                new Translation2d(-24 / 2.0, -24 / 2.0)
        );
        estimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroscopeRotation(),
                swerveDrive.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.5, 0.5, 0.5)
        );

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants TODO tune PID
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants TODO tune PID
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                this // Reference to this subsystem to set requirements

        );
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Fieldx", swerveDrive.getFieldVelocity().vxMetersPerSecond);
//        SmartDashboard.putNumber("Fieldy", swerveDrive.getFieldVelocity().vyMetersPerSecond);
//
//        SmartDashboard.putNumber("Fieldo", swerveDrive.getFieldVelocity().omegaRadiansPerSecond);

//        swerveDrive.updateOdometry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), chassisSpeeds.omegaRadiansPerSecond, false, true);
    }

    public void rotate(double rotation) {
        Translation2d currentTranslation = swerveDrive.getPose().getTranslation();

        swerveDrive.drive(currentTranslation, rotation, true, true);
    }

    public Rotation2d getAngle() {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getFieldVelocity();
    }
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics; 
    }
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }
    public float getNavxYaw() { return m_navx.getYaw(); }
    public float getNavxPitch() { return m_navx.getPitch(); }
    public SwerveDrivePoseEstimator getEstimator() {
        return estimator;
    }


    public SwerveController getSwerveController() {
        return swerveDrive.getSwerveController();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }
    public static Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - m_navx.getYaw());
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
    public void stop() {
        swerveDrive.lockPose();
    }
    // public void addTrajectory(PathPlannerTrajectory m_trajectory) {
    //     field2d.getObject("traj").setTrajectory(m_trajectory);
    // }

    public void setPosition(Pose2d position) {
        zeroGyro();
        estimator.resetPosition(
                getGyroscopeRotation(),
                swerveDrive.getModulePositions(),
                position
        );
    }
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
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

    public Pose2d getPosition() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory)
    {
        swerveDrive.postTrajectory(trajectory);
    }


    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake)
    {
        swerveDrive.setMotorIdleMode(brake);
    }


    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }


    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }



    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock()
    {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch()
    {
        return swerveDrive.getPitch();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        swerveDrive.setModuleStates(states, true);
    }

    private void logger() {
        log("Pose2D", getPose());
        log("Position", getPosition());
        log("Current Module Positions", swerveDrive.getModulePositions());
        log("Field Velocity", getChassisSpeeds());
        log("Robot Velocity", getRobotVelocity());
    }



}

