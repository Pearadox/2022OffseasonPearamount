// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int TIMEOUT = 30;
    public static final double hubX = 8.2296;
    public static final double hubY = 4.1148;

    public static final int PDH_ID = 10;

    public static final class SwerveConstants{
        //Drivetrain motor/encoder IDs
        public static final int LEFT_FRONT_DRIVE_ID = 1;
        public static final int RIGHT_FRONT_DRIVE_ID = 2;
        public static final int LEFT_BACK_DRIVE_ID = 3;
        public static final int RIGHT_BACK_DRIVE_ID = 4;
        
        public static final int LEFT_FRONT_TURN_ID = 5;
        public static final int RIGHT_FRONT_TURN_ID = 6;
        public static final int LEFT_BACK_TURN_ID = 7;
        public static final int RIGHT_BACK_TURN_ID = 8;
        
        public static final int LEFT_FRONT_CANCODER_ID = 11;
        public static final int RIGHT_FRONT_CANCODER_ID = 12;
        public static final int LEFT_BACK_CANCODER_ID = 13;
        public static final int RIGHT_BACK_CANCODER_ID = 14;

        public static final int PIGEON_ID = 15;

        //Drivetrain characteristics
        public static final double LEFT_FRONT_OFFSET = 75.498;
        public static final double RIGHT_FRONT_OFFSET = 267.363;
        public static final double LEFT_BACK_OFFSET = 174.287;
        public static final double RIGHT_BACK_OFFSET = 186.855;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
        public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
        public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
        public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
        public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
        public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
        public static final double KP_TURNING = 0.5;

        public static final double DRIVETRAIN_MAX_SPEED = 4.0;
        public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

        //Teleop constraints
        public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.35;
        public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

        //Auton constraints
        public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 2.0;
        public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
        public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
        public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

        public static final double AUTO_kP_FRONT = 0.4;
        public static final double AUTO_kP_SIDE = 0.4;
        public static final double AUTO_kP_TURN = 2.4;

        public static final TrapezoidProfile.Constraints AUTO_TURN_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                AUTO_DRIVE_MAX_ANGULAR_SPEED,
                AUTO_DRIVE_MAX_ANGULAR_ACCELERATION);

        //Swerve Kinematics
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(19.75);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );
    }

    public static final class IntakeConstants{
        public static final int INTAKE_ID = 24;
        public static final int INTAKE_TOGGLE_ID = 25;

        public static final double INTAKE_DEPLOY_HOLD = 0.02;
        public static final double INTAKE_STOW_HOLD = -0.02;
        public static final double INTAKE_DEPLOY_SPEED = 1.0;
        public static final double INTAKE_STOW_SPEED = -1.0;
  
        public static final double INTAKE_DEPLOY_TIME = 0.55;
        public static final double INTAKE_STOW_TIME = 0.85;
    }

    public static final class TransportConstants{
        public static final int TOP_TRANSPORT_ID = 22;
        public static final int FEEDER_ID = 32;
    }

    public static final class ShooterConstants{
        public static final int LEFT_SHOOTER_ID = 30;
        public static final int RIGHT_SHOOTER_ID = 31;

        public static final double MAXPERCENT = 0.4;
        public static final double ShooterAdjust = 1; //1.165
        
        public static final double kS = 0.0; //0.85049
        public static final double kV = 0.11404;
        public static final double kP = 0.1; //0.2738
    }
}
