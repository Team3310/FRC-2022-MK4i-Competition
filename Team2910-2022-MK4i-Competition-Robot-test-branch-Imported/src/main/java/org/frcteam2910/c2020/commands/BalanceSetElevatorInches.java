package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.BalanceElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceSetElevatorInches extends CommandBase {
    private final BalanceElevator elevator;
    private double inches;

    public BalanceSetElevatorInches(BalanceElevator elevator, double inches) {
        this.elevator = elevator;
        this.inches = inches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setBalanceElevatorMotionMagicPositionAbsoluteRight(inches);
        elevator.setBalanceElevatorMotionMagicPositionAbsoluteLeft(inches);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}