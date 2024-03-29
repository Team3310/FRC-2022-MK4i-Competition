package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class HangarTwoBall extends AutonCommandBase {

    public HangarTwoBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public HangarTwoBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {

        resetRobotPose(container, trajectories.get_HangarFourBallPartOne());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        addCommands(
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_HangarFourBallPartOne()),
                        new ShooterShootWithHoodAuton(shooter, 1900, 30.7),
                        new IntakeLiftSetAngle(intake, 0),
                        new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_AUTO_RPM)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.2),
                        new LimelightAdjustAuto(drive)
                ),
                new FeedBalls(intake, indexer, drive, shooter, Constants.AUTON_INDEXER_RPM)
        );
    }
}
