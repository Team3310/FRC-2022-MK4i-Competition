package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.BalanceElevator.BalanceControlMode;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class BalanceControlJoysticks extends CommandBase {
    private BalanceElevator elevator;
    private Axis YAxis;


    public BalanceControlJoysticks(BalanceElevator elevator, Axis YAxis) {
        this.YAxis = YAxis;

        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = YAxis.get(true)*0.2;

        if(Math.abs(speed) > Constants.MIN_BALANCE_ELEVATOR_PERCENT_BUS){
            elevator.setBalanceElevatorSpeed(speed);
        }
        else{
            if(elevator.getControlMode() == BalanceControlMode.MANUAL){
                elevator.setHoldBalanceElevator();
            }
        }

        
    }

    @Override
    public void end(boolean interrupted) {
   //     elevator.setHoldBalanceElevator();
    }

}
