package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerSetRPM extends CommandBase {
    private final Indexer indexer;
    private double RPM;

    public IndexerSetRPM(Indexer indexer, double RPM) {
        this.indexer = indexer;
        this.RPM = RPM;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.setIndexerRPM(RPM);
    }

    @Override
    public boolean isFinished() {
        indexer.setHoldIndexer();
        return true;
    }
}