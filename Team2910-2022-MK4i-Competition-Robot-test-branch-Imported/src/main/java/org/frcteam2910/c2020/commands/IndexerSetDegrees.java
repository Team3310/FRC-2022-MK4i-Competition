package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerSetDegrees extends CommandBase {
    private final Indexer indexer;
    private double poseDeg;

    public IndexerSetDegrees(Indexer indexer, double poseDeg) {
        this.indexer = indexer;
        this.poseDeg = poseDeg;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.setIndexerPositionAbsolute(poseDeg);
    }

    @Override
    public boolean isFinished() {
        indexer.setHoldIndexer();
        return true;
    }
}