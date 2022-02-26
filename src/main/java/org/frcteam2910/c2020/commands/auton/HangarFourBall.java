package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.ShooterShootAllField;
import org.frcteam2910.c2020.commands.ShooterShootWithHood;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HangarFourBall extends AutonCommandBase {

    public HangarFourBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public HangarFourBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {

        resetRobotPose(container, trajectories.get_HangarFourBallPartOne());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        addCommands(
            new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
            new WaitCommand(.5),
            new FollowTrajectoryCommand(drive, trajectories.get_HangarFourBallPartOne()),
            new AllFieldAuton(shooter, drive),
            new WaitCommand(0.5),
            new FeedBalls(intake, indexer),
            new WaitCommand(0.5),
            new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
            new FollowTrajectoryCommand(drive, trajectories.get_HangarFourBallPartTwo()),
            new FollowTrajectoryCommand(drive, trajectories.get_StartPosition1ToBall4()),
            new AllFieldAuton(shooter, drive),
            new WaitCommand(0.5),
            new FeedBalls(intake, indexer),
            new WaitCommand(0.5),
            new IntakeIndexerHalt(intake, indexer)
        );
    }
}
