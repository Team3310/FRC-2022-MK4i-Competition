package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class ClimbControlJoysticks extends CommandBase {
    private ClimbElevator elevator;
    private Axis YAxis;


    public ClimbControlJoysticks(ClimbElevator elevator, Axis YAxis) {
        this.YAxis = YAxis;

        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
  //      if(elevator.getControlMode() != ClimbElevator.ClimbControlMode.MOTION_MAGIC) {
            double speed = YAxis.get(true);

            if (Math.abs(speed) > Constants.MIN_CLIMB_ELEVATOR_PERCENT_BUS) {
                elevator.setElevatorSpeed(speed);
            } else {
                elevator.setHoldElevator();
            }
  //      }
    }

//    @Override
//    public void end(boolean interrupted) {
//        elevator.setHoldElevator();
//    }

}
