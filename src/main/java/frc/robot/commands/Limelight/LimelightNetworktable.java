package frc.robot.commands.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightNetworktable extends CommandBase {
    private final NetworkTable limelightTable;

    public LimelightNetworktable() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void execute() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight Data");

        for (String key : limelightTable.getKeys()) {
            NetworkTableEntry entry = limelightTable.getEntry(key);
            tab.add(key, entry.getValue());
        }
        
    }
}
