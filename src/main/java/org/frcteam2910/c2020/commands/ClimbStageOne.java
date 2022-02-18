package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.subsystems.BalanceElevator.BalanceControlMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimbStageOne extends SequentialCommandGroup {

    public ClimbStageOne(Shooter shooter, BalanceElevator balanceElevator, ClimbElevator climbElevator) {
        addCommands(
             new HoodSetAngle(shooter, Constants.HOOD_MIN_ANGLE_DEGREES),
             new ClimbSetElevatorInches(climbElevator, Constants.ELEVATOR_STAGE_ONE_INCHES),
             new WaitCommand(0.5)
            //  new BalanceSetElevatorInches(balanceElevator, Constants.BALANCE_ELEVATOR_MAX_INCHES),
            //  new ClimbSetElevatorInches(climbElevator, Constants.ELEVATOR_STAGE_TWO_INCHES)
         );
    }
}