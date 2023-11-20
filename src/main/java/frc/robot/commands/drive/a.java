package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.DigitalSensor;


public class a extends CommandBase implements SubsystemLogging {
    private DigitalSensor sens;
    public a(DigitalSensor sens) {
        this.sens = sens;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(sens);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        log("Sens0", sens.getSens0());
        log("sens1", sens.getSens1());

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
