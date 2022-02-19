package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;

public class IntakeIndexerStop extends SequentialCommandGroup {

    public IntakeIndexerStop(Intake intake, Indexer indexer) {
        addCommands(
                new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                new IndexerBallStop(indexer)
        );
    }
}