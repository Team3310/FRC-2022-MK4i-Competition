package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.BalanceElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceSetElevatorDeltaInches extends CommandBase {
    private final BalanceElevator elevator;
    private double deltaInches;
    double inches;

    public BalanceSetElevatorDeltaInches(BalanceElevator elevator, double deltaInches) {
        this.elevator = elevator;
        this.deltaInches = deltaInches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        inches = elevator.getBalanceElevatorInches() + deltaInches;
        elevator.setBalanceElevatorMotionMagicPositionAbsolute(inches);
    }

    @Override
    public boolean isFinished() {
        elevator.setHoldBalanceElevator();
        return true;
    }
}