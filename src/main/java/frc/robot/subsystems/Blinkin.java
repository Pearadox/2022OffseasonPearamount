// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Blinkin extends SubsystemBase {
  Spark m_blinkin = new Spark(0);
  double value = -1;

  private static final Blinkin blinkin = new Blinkin();

  public static Blinkin getInstance(){
    return blinkin;
  }
  
  /** Creates a new Blinkin. */
  public Blinkin() {}

  public void activate(){
    double target = RobotContainer.shooter.llTable.getEntry("tv").getDouble(0);
    if(RobotContainer.intake.getIntakeCurrent() > 10){
      m_blinkin.set(0.67);
    }
    else{
    if(target != 0){
      if(RobotContainer.transport.hasBall()){
        m_blinkin.set(0.75); //dark green
      }
      else{
        m_blinkin.set(0.73); //lime
      }
    }
    else{
      if(RobotContainer.transport.hasBall()){
        m_blinkin.set(0.59); //dark red
      }
      else{
        m_blinkin.set(0.61); //red
      }
    }
  }
  }

  public void valueInc(){
    if(value < 1){
      value += 0.01;
    }
    else{
      value = -1;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Blinkin", value);
  }
}
