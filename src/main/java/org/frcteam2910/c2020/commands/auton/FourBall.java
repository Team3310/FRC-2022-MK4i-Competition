package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourBall extends AutonCommandBase {
    public FourBall(RobotContainer container, AutonomousTrajectories trajectories) {
        resetRobotPose(container, trajectories.get_StartPosition1ToBall2());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new WaitCommand(.5),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall1()),
            new AllFieldAuton(container.getShooter(), container.getDrivetrainSubsystem()), 
            new WaitCommand(.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(.5),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall2()),
            new WaitCommand(.3),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall3()),
            new WaitCommand(.25),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall4()),
            new WaitCommand(.25),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToShoot()),
            new AllFieldAuton(container.getShooter(), container.getDrivetrainSubsystem()),
            new WaitCommand(.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer())
        );
    }
}
