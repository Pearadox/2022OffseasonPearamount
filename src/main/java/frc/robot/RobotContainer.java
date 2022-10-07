// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.drivers.EForwardableConnections;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterRampUpVoltage;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.TransportIn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Shooter.Mode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static final Transport transport = Transport.getInstance();
  public static final Shooter shooter = Shooter.getInstance();

  public static final XboxController controller = new XboxController(0);
  private final JoystickButton resetHeading_B = new JoystickButton(controller, XboxController.Button.kB.value);
  private final JoystickButton toggleMode_A = new JoystickButton(controller, XboxController.Button.kA.value);
  private final JoystickButton X = new JoystickButton(controller, XboxController.Button.kX.value);
  private final JoystickButton lowShoot_Y = new JoystickButton(controller, XboxController.Button.kY.value);
  private final JoystickButton toggleIntake_LB = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  private final JoystickButton shoot_RB = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  private final JoystickButton toggleSystem_Start = new JoystickButton(controller, XboxController.Button.kStart.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    portForwarding();
    configureButtonBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    shooter.setDefaultCommand(new ShooterRampUpVoltage());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleMode_A.whenPressed(() -> shooter.toggleMode());
    resetHeading_B.whenPressed(() -> drivetrain.zeroHeading());
    // lowShoot_Y.whileHeld(new InstantCommand(() -> shooter.setMode(Mode.kFixedLow))
    // .andThen(new RunCommand(() -> transport.feederShoot())))
    // .whenReleased(new InstantCommand(() -> shooter.setMode(Mode.kAuto))
    // .andThen(new InstantCommand(() -> transport.stop())));
    lowShoot_Y.whileHeld(new RunCommand(() -> intake.setToggler(-1.0))).whenReleased(new InstantCommand(() -> intake.setToggler(0)));
    X.whileHeld(new RunCommand(() -> intake.setToggler(1.0))).whenReleased(new InstantCommand(() -> intake.setToggler(0)));
    toggleIntake_LB.whenPressed(new ToggleIntake().withTimeout(0.4));
    shoot_RB.whileHeld(new Shoot())
      .whenReleased(new InstantCommand(() -> transport.stop()));
    toggleSystem_Start.toggleWhenPressed(
      new RunCommand(() -> transport.stop())
      .alongWith(new RunCommand(() -> shooter.setSpeed(0)),
                new RunCommand(() -> intake.stop())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
    PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
    ProfiledPIDController turnController = new ProfiledPIDController(
      SwerveConstants.AUTO_kP_TURN, 0, 0, SwerveConstants.AUTO_TURN_CONTROLLER_CONSTRAINTS);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory trajFiveA = PathPlanner.loadPath("FiveBallA", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);
    PPSwerveControllerCommand FiveBallA = new PPSwerveControllerCommand(
      trajFiveA, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );

    PathPlannerTrajectory trajFiveB = PathPlanner.loadPath("FiveBallB", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);
    PPSwerveControllerCommand FiveBallB = new PPSwerveControllerCommand(
      trajFiveB, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );

    PathPlannerTrajectory trajFiveC = PathPlanner.loadPath("FiveBallC", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);
    PPSwerveControllerCommand FiveBallC = new PPSwerveControllerCommand(
      trajFiveC, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );

    PathPlannerTrajectory trajFiveD = PathPlanner.loadPath("FiveBallD", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);
    PPSwerveControllerCommand FiveBallD = new PPSwerveControllerCommand(
      trajFiveD, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );

    PathPlannerTrajectory trajFiveE = PathPlanner.loadPath("FiveBallE", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);
    PPSwerveControllerCommand FiveBallE = new PPSwerveControllerCommand(
      trajFiveE, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );

    PathPlannerState initialFiveBallState = trajFiveA.getInitialState();
    Pose2d initialFiveBallPose = new Pose2d(
      trajFiveA.getInitialPose().getTranslation(),
      initialFiveBallState.holonomicRotation
    );

    SequentialCommandGroup FiveBall = new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetOdometry(initialFiveBallPose)),
      FiveBallA,
      new InstantCommand(() -> drivetrain.stopModules()),
      new Shoot().withTimeout(2),
      FiveBallB,
      new InstantCommand(() -> drivetrain.stopModules()),
      new Shoot().withTimeout(2),
      FiveBallC,
      new InstantCommand(() -> drivetrain.stopModules()),
      new WaitCommand(0.1),
      FiveBallD,
      new InstantCommand(() -> drivetrain.stopModules()),
      new WaitCommand(1.5),
      FiveBallE,
      new InstantCommand(() -> drivetrain.stopModules()),
      new Shoot().withTimeout(2)
    );

    return FiveBall;
  }

  private void portForwarding(){
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }
}
