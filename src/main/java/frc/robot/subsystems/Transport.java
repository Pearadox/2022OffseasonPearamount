// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {
  public PearadoxSparkMax topMotor;
  public TalonFX feeder;

  DigitalInput ballDetector = new DigitalInput(0);

  private static final Transport transport = new Transport();

  public static Transport getInstance(){
    return transport;
  }

  /** Creates a new Transport. */
  public Transport() {
    topMotor = new PearadoxSparkMax(TransportConstants.TOP_TRANSPORT_ID, MotorType.kBrushless, IdleMode.kBrake, 20, true);
    feeder = new TalonFX(TransportConstants.FEEDER_ID);
    feeder.setNeutralMode(NeutralMode.Brake);
    feeder.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Transport Current", topMotor.getOutputCurrent());
    SmartDashboard.putNumber("Feeder Current", feeder.getSupplyCurrent());
    SmartDashboard.putBoolean("Has Ball?", hasBall());
  }

  public void setSpeed(double speed){
    topMotor.set(speed/2);
    feeder.set(ControlMode.PercentOutput, speed);
  }

  public void setFeeder(double speed){
    feeder.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    topMotor.set(0);
    feeder.set(ControlMode.PercentOutput, 0);
  }

  public void transportIn(){
    if(!hasBall()){
      topMotor.set(0.45);
    }
    else{
      topMotor.set(0);
    }
  }

  public void transportOut(){
    topMotor.set(-0.85);
  }

  public double getFeederCurrent(){
    return feeder.getSupplyCurrent();
  }

  public double getTopCurrent(){
    return topMotor.getOutputCurrent();
  }

  public void feederShoot(){
    topMotor.set(0.75);
    feeder.set(ControlMode.PercentOutput, 1);
  }

  public void feederHold(){
    if(!hasBall()){
      feeder.set(ControlMode.PercentOutput, -0.8);
    }
    else{
      feeder.set(ControlMode.PercentOutput, 0);
    }
  }

  public boolean hasBall(){
    return !ballDetector.get();
  }
}
