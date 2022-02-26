package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.ClimbSetElevatorInches;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.HoodSetAngle;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.ShooterSetRPM;
import org.frcteam2910.c2020.commands.ShooterShootWithHood;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FiveBall extends AutonCommandBase {

    public FiveBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public FiveBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.get_StartPosition0ToBall1());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new ClimbSetElevatorInches(container.getClimbElevator(), 26),
            new ShooterSetRPM(shooter, 2000),
            new HoodSetAngle(shooter, Constants.IDLE_HOOD_ANGLE),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition0ToBall1()),
            new AllFieldAuton(container.getShooter(), container.getDrivetrainSubsystem()), 
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_ThreeBallPartTwo()),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall1()),
            new WaitCommand(0.25),
            new AllFieldAuton(container.getShooter(), container.getDrivetrainSubsystem()), 
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall2()),
            new WaitCommand(0.3),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToShoot()),
            new AllFieldAuton(container.getShooter(), container.getDrivetrainSubsystem()), 
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer())

        );
    }
}
