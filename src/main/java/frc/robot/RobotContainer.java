// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.drivers.EForwardableConnections;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.BlinkinState;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterRampUpVoltage;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Blinkin;
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
  public static final Blinkin blinkin = Blinkin.getInstance();

  public static final XboxController controller = new XboxController(0);
  private final JoystickButton resetHeading_B = new JoystickButton(controller, XboxController.Button.kB.value);
  private final JoystickButton toggleMode_A = new JoystickButton(controller, XboxController.Button.kA.value);
  private final JoystickButton X = new JoystickButton(controller, XboxController.Button.kX.value);
  private final JoystickButton lowShoot_Y = new JoystickButton(controller, XboxController.Button.kY.value);
  // private final JoystickButton toggleIntake_LB = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  private final JoystickButton shoot_RB = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  private final JoystickButton toggleSystem_Start = new JoystickButton(controller, XboxController.Button.kStart.value);

  private SendableChooser<String> auton = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    portForwarding();
    loadCommands();
    configureButtonBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    shooter.setDefaultCommand(new ShooterRampUpVoltage());
    blinkin.setDefaultCommand(new BlinkinState());
    
    SmartDashboard.putData("Auton Chooser", auton);
    auton.setDefaultOption("Nothing", "Nothing");
    auton.addOption("TwoBall", "TwoBall");
    auton.addOption("Taxi", "Taxi");
    auton.addOption("FiveBall", "FiveBall");
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
    lowShoot_Y.whileHeld(new InstantCommand(() -> shooter.setMode(Mode.kFixedLow))
    .andThen(new RunCommand(() -> transport.feederShoot())))
    .whenReleased(new InstantCommand(() -> shooter.setMode(Mode.kAuto))
    .andThen(new InstantCommand(() -> transport.stop())));
    // lowShoot_Y.whileHeld(new RunCommand(() -> intake.setToggler(-1.0))).whenReleased(new InstantCommand(() -> intake.setToggler(0)));
    X.whenPressed(new InstantCommand(() -> blinkin.valueInc()));
    // toggleIntake_LB.whenPressed(new ToggleIntake().withTimeout(0.4));
    shoot_RB.whileHeld(new Shoot())
      .whenReleased(new InstantCommand(() -> transport.stop()));
    toggleSystem_Start.toggleWhenPressed(
      new RunCommand(() -> transport.stop())
      .alongWith(new RunCommand(() -> shooter.setSpeed(0)),
                new RunCommand(() -> intake.stop())));
  }

  private static ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<>();
  private static ArrayList<String> paths = new ArrayList<>();
  private static ArrayList<Command> swerveCommands = new ArrayList<>();
  private PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
  private PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
  private ProfiledPIDController turnController = new ProfiledPIDController(
    SwerveConstants.AUTO_kP_TURN, 0, 0, SwerveConstants.AUTO_TURN_CONTROLLER_CONSTRAINTS);

  private Command makeSwerveCommand(String path){
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectories.get(paths.indexOf(path)), 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates, 
      drivetrain
    );
        
    return command;
  }

  private void loadCommands() {
    paths.add("FiveBallA");
    paths.add("FiveBallB");
    paths.add("FiveBallC");
    paths.add("FiveBallD");
    paths.add("FiveBallE");

    turnController.enableContinuousInput(-Math.PI, Math.PI);

    for(String path : paths) {
      trajectories.add(PathPlanner.loadPath(path, SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));
      swerveCommands.add(makeSwerveCommand(path));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    PathPlannerState initialFiveBallState = trajectories.get(0).getInitialState();
    Pose2d initialFiveBallPose = new Pose2d(
      trajectories.get(0).getInitialPose().getTranslation(),
      initialFiveBallState.holonomicRotation
    );

    var FiveBall = new InstantCommand(() -> shooter.setMode(Mode.kAuto))
    .andThen(new InstantCommand(() -> transport.stop()))
    .andThen(new InstantCommand(() -> intake.intakeIn(0.5)))
    .andThen(makeSwerveCommand("FiveBallA"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new Shoot().withTimeout(2))
    .andThen(new InstantCommand(() -> transport.stop()))
    .andThen(makeSwerveCommand("FiveBallB"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new Shoot().withTimeout(1.3))
    .andThen(new InstantCommand(() -> transport.stop()))
    .andThen(makeSwerveCommand("FiveBallC"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new WaitCommand(0.1))
    .andThen(makeSwerveCommand("FiveBallD"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new WaitCommand(1))
    .andThen(makeSwerveCommand("FiveBallE"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new Shoot().withTimeout(2))
    .andThen(new InstantCommand(() -> transport.stop()));

    var Taxi = makeSwerveCommand("FiveBallA")
    .andThen(new InstantCommand(() -> drivetrain.stopModules()));

    var TwoBall = new InstantCommand(() -> shooter.setMode(Mode.kAuto))
    .andThen(new InstantCommand(() -> transport.stop()))
    .andThen(new InstantCommand(() -> intake.intakeIn(0.5)))
    .andThen(makeSwerveCommand("FiveBallA"))
    .andThen(new InstantCommand(() -> drivetrain.stopModules()))
    .andThen(new Shoot().withTimeout(2))
    .andThen(new InstantCommand(() -> transport.stop()));

    if(auton.getSelected().equals("FiveBall")){
      drivetrain.resetAllEncoders();
      drivetrain.resetOdometry(initialFiveBallPose);
      return FiveBall;
    }
    else if(auton.getSelected().equals("TwoBall")){
      drivetrain.resetAllEncoders();
      drivetrain.resetOdometry(initialFiveBallPose);
      return TwoBall;
    }
    else if(auton.getSelected().equals("Taxi")){
      drivetrain.resetAllEncoders();
      drivetrain.resetOdometry(initialFiveBallPose);
      return Taxi;
    }
    else{
      return new InstantCommand(() -> drivetrain.stopModules());
    }
  }

  private void portForwarding(){
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }
}
