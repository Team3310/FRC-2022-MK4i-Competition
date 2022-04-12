package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;

public class FeedBalls extends SequentialCommandGroup {

    public FeedBalls(Intake intake, Indexer indexer, DrivetrainSubsystem drive, Shooter shooter, double RPM) {
        addCommands(
                new ChangeDriveMode(drive, DrivetrainSubsystem.DriveControlMode.LIMELIGHT_LOCKED),
                new InstantCommand(() -> shooter.setIsShooting(true)),
                new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                new IndexerSetRPM(indexer, RPM)
        );
    }
}