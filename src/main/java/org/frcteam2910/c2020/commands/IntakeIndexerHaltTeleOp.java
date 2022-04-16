package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.LED;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.subsystems.LED.ShooterStatusEnum;

public class IntakeIndexerHaltTeleOp extends SequentialCommandGroup {

    public IntakeIndexerHaltTeleOp(Intake intake, Indexer indexer, Shooter shooter, DrivetrainSubsystem drive, LED led) {
        addCommands(
                new InstantCommand(() -> shooter.setIsShooting(false)),
                new IntakeSetRPM(intake, 0),
                new IndexerSetSpeed(indexer, 0),
                new ChangeDriveMode(drive, DrivetrainSubsystem.DriveControlMode.JOYSTICKS),
                new InstantCommand(() -> led.setShooterStatus(ShooterStatusEnum.OFF))
        );
    }
}