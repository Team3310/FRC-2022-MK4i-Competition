package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.BalanceElevator.BalanceControlMode;
import org.frcteam2910.c2020.subsystems.ClimbElevator.ClimbControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ActivateZeroMode extends CommandBase {
    private final ClimbElevator cElevator;
    private final BalanceElevator bElevator;

    public ActivateZeroMode(ClimbElevator cElevator, BalanceElevator bElevator) {
        this.cElevator = cElevator;
        this.bElevator = bElevator;
        addRequirements(this.cElevator);
        addRequirements(this.bElevator);
    }

    @Override
    public void initialize() {
        cElevator.setControlMode(ClimbControlMode.ZERO);
        bElevator.setControlMode(BalanceControlMode.ZERO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}