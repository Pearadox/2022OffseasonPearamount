// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoAim extends CommandBase {
  public NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  double kS = 0.078;
  double kP = 0.007;

  /** Creates a new AutoAim. */
  public AutoAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    llTable.getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = llTable.getEntry("tx").getDouble(0);
    double target = llTable.getEntry("tv").getDouble(0);

    if(RobotContainer.shooter.getMode() != RobotContainer.shooter.getLowMode()){
      if(target != 0){
        if(Math.abs(error) > 0.4){
          RobotContainer.drivetrain.swerveDrive(0, 0, -(Math.signum(error) * kS + kP * error), true, false);
        }
      }
      else{
        error = RobotContainer.drivetrain.getHubAngle() - RobotContainer.drivetrain.getPose().getRotation().getDegrees();
        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        RobotContainer.drivetrain.swerveDrive(0, 0, Math.signum(error) * kS + kP * error, true, false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}