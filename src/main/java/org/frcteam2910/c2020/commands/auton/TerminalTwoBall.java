package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.ShooterShootAllFieldAuto;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TerminalTwoBall extends AutonCommandBase {
    public TerminalTwoBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public TerminalTwoBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.get_StartPosition0ToBall1());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_StartPosition0ToBall1()),
                        new ShooterShootAllFieldAuto(shooter),
                        new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_AUTO_RPM)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.75),
                        new AllFieldAuton(shooter, drive)
                ),
                new FeedBalls(intake, indexer)
        );
    }
}
