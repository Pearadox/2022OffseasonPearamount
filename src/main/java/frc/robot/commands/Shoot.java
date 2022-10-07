// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends ParallelCommandGroup {
  /** Creates a new Shoot. */
  public Shoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoAim().until(() -> Math.abs(RobotContainer.shooter.getSpeed() - RobotContainer.shooter.getTarget()) < 0.5
                          && Math.abs(RobotContainer.shooter.llTable.getEntry("tx").getDouble(0)) < 0.5).withTimeout(2)
                          .andThen(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.intake.setDeployed(false)),
      new WaitCommand(0.5)
        .andThen(new RunCommand(RobotContainer.transport::feederShoot, RobotContainer.transport)) 
    );
  }
}
