package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.BalanceElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceElevatorSetSpeed extends CommandBase {
    private final BalanceElevator elevator;
    private double speed;

    public BalanceElevatorSetSpeed(BalanceElevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setBalanceElevatorSpeed(speed);
    }

    @Override
    public boolean isFinished() {
   //     elevator.setHoldBalanceElevator();
        return true;
    }
}