package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerSetDeltaDegrees extends CommandBase {
    private final Indexer indexer;
    private double deltaDeg;
    private double deg;

    public IndexerSetDeltaDegrees(Indexer indexer, double deltaDeg) {
        this.indexer = indexer;
        this.deltaDeg = deltaDeg;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        deg = indexer.getIndexerDegrees() + deltaDeg;
        indexer.setIndexerMotionMagicPositionAbsolute(deg);
    }

    @Override
    public boolean isFinished() {
        indexer.setHoldIndexer();
        return true;
    }
}