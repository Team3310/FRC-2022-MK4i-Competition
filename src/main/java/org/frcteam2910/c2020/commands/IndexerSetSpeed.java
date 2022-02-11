package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerSetSpeed extends CommandBase {
    private final Indexer indexer;
    private double speed;

    public IndexerSetSpeed(Indexer indexer, double speed) {
        this.indexer = indexer;
        this.speed = speed;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.setIndexerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        indexer.setHoldIndexer();
        return true;
    }
}