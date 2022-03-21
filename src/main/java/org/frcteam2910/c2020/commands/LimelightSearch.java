package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;

public class LimelightSearch extends CommandBase {
    private DrivetrainSubsystem drive;
    private boolean toTheRight;

    public LimelightSearch(DrivetrainSubsystem drive, boolean toTheRight) {
        this.drive = drive;
        this.toTheRight = toTheRight;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT_SEARCH);
        drive.setRotationRight(toTheRight);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
