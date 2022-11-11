package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ExperimentalEjectBalls extends SequentialCommandGroup {

    public ExperimentalEjectBalls(Intake intake, Indexer indexer) {
        addCommands(
                new IntakeSetSpeed(intake, -1.0),
                new IndexerSetSpeed(indexer, -1.0)
        );
    }
}