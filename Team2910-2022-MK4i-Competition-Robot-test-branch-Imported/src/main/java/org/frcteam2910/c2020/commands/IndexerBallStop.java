// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerBallStop extends CommandBase {
  
  private Indexer indexer;

  public IndexerBallStop(Indexer indexer) {
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!indexer.getIndexerSensor())
      indexer.setIndexerSpeed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.getIndexerSensor();
  }
}
