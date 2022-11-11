// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2020.subsystems;

import java.awt.Color;

//doesn't look like there is a library for CANdle
//import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.common.robot.UpdateManager;


/** Add your docs here. */
public class LED extends SubsystemBase implements UpdateManager.Updatable{
  //private CANdle led = new CANdle(0);
  private Shooter shooter;
  private Indexer indexer;

  public enum ShooterStatusEnum {
    LOCKED, 
    SEARCHING, 
    NOTARGET,
    OFF
  }

  public enum BallsLoadedEnum {
    ZERO, 
    ONE
  }

  private ShooterStatusEnum shooterStatus = ShooterStatusEnum.NOTARGET;
  private BallsLoadedEnum ballsLoaded = BallsLoadedEnum.ZERO;
  
  // private final int shooterStatusStart = 0;
  // private final int shooterStatusLength = 4;
  // private final int ballsLoadedStart = 4;
  // private final int ballsLoadedLength = 4;

  private final Color shooterStatus_Locked_Color = Color.WHITE;
  private final Color shooter_Searching_Color = Color.MAGENTA;
  private final Color shooterStatus_Manual_Color = Color.RED;
  private final Color ballsLoaded_ZERO_Color = Color.RED;
  private final Color ballsLoaded_ONE_Color = Color.GREEN;

  public LED(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    //CANdleConfiguration config = new CANdleConfiguration();
    // config.stripType = LEDStripType.GRB; // set the strip type to GRB
    //led.configAllSettings(config);
    //led.setLEDs(0, 0, 0);
  }

  public void setShooterStatus(ShooterStatusEnum status){
    this.shooterStatus = status;
  }

  public void setBallsLoadedStatus(BallsLoadedEnum status){
    this.ballsLoaded = status;
  }

  public String getShooterStatus(){
    switch(shooterStatus){
      case LOCKED : return "Locked";
      case SEARCHING : return "Searching";
      case NOTARGET : return "No Traget";
      default : return "Not Tracking";
    }
  }

  public String getBallsLoadedStatus(){
    switch(ballsLoaded){
      case ZERO : return "Zero";
      default : return "one";
    }
  }

  public void setShooterStatusLEDs(){
    int r, g, b;
    switch(shooterStatus){
      case LOCKED :
        r = shooterStatus_Locked_Color.getRed();
        g = shooterStatus_Locked_Color.getGreen();
        b = shooterStatus_Locked_Color.getBlue();
        break;
      case SEARCHING :
        r = shooter_Searching_Color.getRed();
        g = shooter_Searching_Color.getGreen();
        b = shooter_Searching_Color.getBlue();
        break;
      default :
        r = shooterStatus_Manual_Color.getRed();
        g = shooterStatus_Manual_Color.getGreen();
        b = shooterStatus_Manual_Color.getBlue();
    }
    //led.setLEDs(r, g, b);
  }

  public void setBallsLoadedLEDs(){
    int r, g, b;
    switch(ballsLoaded){
      case ZERO : 
        r = ballsLoaded_ZERO_Color.getRed();
        g = ballsLoaded_ZERO_Color.getGreen();
        b = ballsLoaded_ZERO_Color.getBlue();
        break;
      default :
        r = ballsLoaded_ONE_Color.getRed();
        g = ballsLoaded_ONE_Color.getGreen();
        b = ballsLoaded_ONE_Color.getBlue(); 
    }
    //led.setLEDs(r, g, b);
  }

  @Override
  public void update(double time, double dt) {
    shooterStatus = shooter.isLocked();
    if(indexer.getIndexerSensor())
      ballsLoaded = BallsLoadedEnum.ONE;
    else
      ballsLoaded = BallsLoadedEnum.ZERO;  
    if(shooterStatus != ShooterStatusEnum.OFF)
      setShooterStatusLEDs();
     else
      setBallsLoadedLEDs();  
  }
}