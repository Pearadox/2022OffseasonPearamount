// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID);
  private PearadoxSparkMax intakeToggler;

  private static final Intake intake = new Intake();

  public static Intake getInstance(){
    return intake;
  }
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);
    intakeMotor.setInverted(true);
    intakeToggler = new PearadoxSparkMax(IntakeConstants.INTAKE_TOGGLE_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Toggle Current", getToggleCurrent());
  }

  public void intakeIn(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getIntakeCurrent(){
    return intakeMotor.getSupplyCurrent();
  }

  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void toggleIntake(double speed){
    intakeToggler.set(speed);
  }

  public double getToggleCurrent(){
    return intakeToggler.getOutputCurrent();
  }
}
