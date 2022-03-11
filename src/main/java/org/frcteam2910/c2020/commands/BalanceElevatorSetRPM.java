package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.Indexer;

public class BalanceElevatorSetRPM extends CommandBase {
    private final Indexer indexer;
    private double RPM;

    public BalanceElevatorSetRPM(Indexer indexer, double RPM) {
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
        return true;
    }
}