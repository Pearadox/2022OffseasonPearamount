// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TransportIn extends CommandBase {
  double start;
  /** Creates a new TransportIn. */
  public TransportIn() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
    RobotContainer.transport.clearBall();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.transport.transportIn();
    // RobotContainer.transport.feederHold();
    if(Timer.getFPGATimestamp() - start > 2 && RobotContainer.transport.getTopCurrent() > 7){
      RobotContainer.transport.detectBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.transport.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}