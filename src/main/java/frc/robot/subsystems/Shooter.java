// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LerpTable;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooter;
  private TalonFX rightShooter;
  private SupplyCurrentLimitConfiguration limitCurrent;

  private MedianFilter tyFilter = new MedianFilter(5);
  public NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  private LerpTable shooterLerp = new LerpTable();
  private double target = 0;
  private Mode mode = Mode.kAuto;
  private SlewRateLimiter shooterLimiter = new SlewRateLimiter(10);
  private double ty;

  public enum Mode {
    kAuto, kFixedHigh, kFixedLow;
  }

  private static final Shooter shooter = new Shooter();

  public static Shooter getInstance(){
    return shooter;
  }
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter = new TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
    rightShooter = new TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);
    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
    leftShooter.setInverted(false);
    rightShooter.setInverted(false);

    SmartDashboard.putNumber("SetVoltage", 4.5);
    SmartDashboard.putNumber("Lerp Target", target);
    limitCurrent = new SupplyCurrentLimitConfiguration(true, 45, 45, 1);
    leftShooter.configSupplyCurrentLimit(limitCurrent);
    rightShooter.configSupplyCurrentLimit(limitCurrent);
    leftShooter.enableVoltageCompensation(true);
    rightShooter.enableVoltageCompensation(true);
    leftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);
    rightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);

    shooterLerp.addPoint(4.65, ShooterConstants.ShooterAdjust * (4.15 - ShooterConstants.kS)/ ShooterConstants.kV); 
    shooterLerp.addPoint(0.66, ShooterConstants.ShooterAdjust * (4.25 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-2.68, ShooterConstants.ShooterAdjust * (4.44 - ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-5.78, ShooterConstants.ShooterAdjust * (4.61 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-8.27, ShooterConstants.ShooterAdjust * (4.72 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-10.73, ShooterConstants.ShooterAdjust * (4.83 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-12.64, ShooterConstants.ShooterAdjust * (4.93 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-15.91, ShooterConstants.ShooterAdjust * (5.12 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-17.43, ShooterConstants.ShooterAdjust * (5.32 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-18.60, ShooterConstants.ShooterAdjust * (5.58 - ShooterConstants.kS) / ShooterConstants.kV); 
    shooterLerp.addPoint(-20.30, ShooterConstants.ShooterAdjust * (5.95 - ShooterConstants.kS) / ShooterConstants.kV); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter PercentOutput", getPercentOutput());
      SmartDashboard.putNumber("Shooter Current", leftShooter.getSupplyCurrent());
      SmartDashboard.putNumber("Shooter Temp", leftShooter.getTemperature());
      SmartDashboard.putNumber("Shooter Bus Voltage", leftShooter.getBusVoltage());
      SmartDashboard.putString("Shooter Mode", mode.toString());
      SmartDashboard.putNumber("Shooter RPS", getSpeed());
      
      if (llTable.getEntry("ta").getDouble(0) > 0 && mode == Mode.kAuto) {
        ty = tyFilter.calculate(llTable.getEntry("ty").getDouble(0));
        target = shooterLerp.interpolate(ty);
      } 
      else if (mode == Mode.kFixedLow) {
        target = 3.5 / ShooterConstants.kV;
      } 
      else {
        target = SmartDashboard.getNumber("SetVoltage", 4.5)/ ShooterConstants.kV;
      }
      SmartDashboard.putNumber("Lerp Target", target);
  }

  public void setRightSpeed(double speed){
    rightShooter.set(ControlMode.PercentOutput, speed);
  }

  public void setVoltage(double voltage) {
    var limitedVoltage = shooterLimiter.calculate(voltage);
    leftShooter.set(ControlMode.PercentOutput, limitedVoltage/leftShooter.getBusVoltage());
    rightShooter.set(ControlMode.PercentOutput, limitedVoltage/rightShooter.getBusVoltage());
  }

  public void setSetpoint(double rps) {
    setVoltage(ShooterConstants.kS + ShooterConstants.kV * rps + ShooterConstants.kP * (rps - getSpeed()));
  }

  public void autoSpeed() {
    setSetpoint(target);
  }

  public double getSpeed() {
    return (leftShooter.getSelectedSensorVelocity() + rightShooter.getSelectedSensorVelocity()) / 409.6;
  }

  public double getPercentOutput() {
    return (leftShooter.getMotorOutputPercent() + rightShooter.getMotorOutputPercent())/2;
  }

  //Positive Out, Negative In
  public void setSpeed(double speed){
    leftShooter.set(ControlMode.PercentOutput, speed);
    rightShooter.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    setSpeed(0);
  }

  public void setLeds(int state) {
    llTable.getEntry("ledMode").setNumber(state);
  }

  public Mode getMode(){
    return mode;
  }

  public Mode getLowMode(){
    return Mode.kFixedLow;
  }

  public void toggleMode(){
    if(mode == Mode.kAuto){
      mode = Mode.kFixedHigh;
    }
    else if (mode == Mode.kFixedHigh){
      mode = Mode.kAuto;
    }
    else{
      mode = Mode.kAuto;
    }
  }

  public void setMode(Mode mode){
    this.mode = mode;
  }

  public double getTarget() {
    return target;
  }
}
