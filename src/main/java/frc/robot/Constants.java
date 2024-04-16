// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import cowlib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double deadband = 0.08;
    public static final int currentLimit = 40;
    public static final double slewRate = 20; // lower number for higher center of mass

    public static final class SwervePID {
      public static final double p = 0.12;
      public static final double i = 0;
      public static final double d = 0.0015;
    }

    public static final class SwerveModules {

      // Front Left Module
      public static final SwerveModuleConfig Front_Left_Configuration = new SwerveModuleConfig(
          8,
          1,
          true,
          true,
          false,
          2 * Math.PI,
          .462);

      // Front Right
      public static final SwerveModuleConfig Front_Right_Configuration = new SwerveModuleConfig(
          6,
          7,
          true,
          true,
          false,
          2 * Math.PI,
          0);

      // Back Right
      public static final SwerveModuleConfig Back_Right_Configuration = new SwerveModuleConfig(
          4,
          5,
          true,
          true,
          false,
          2 * Math.PI,
          .759);

      // Back Left
      public static final SwerveModuleConfig Back_Left_Configuration = new SwerveModuleConfig(
          2,
          3,
          true,
          false,
          false,
          2 * Math.PI,
          .158 // 0 to 1
      );
    }

    public static final class ModuleLocations {
      public static final double dist = Units.inchesToMeters(9.25);
      public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
      public static final Translation2d frontLeft = new Translation2d(dist, dist);
      public static final Translation2d frontRight = new Translation2d(dist, -dist);
      public static final Translation2d backLeft = new Translation2d(-dist, dist);
      public static final Translation2d backRight = new Translation2d(-dist, -dist);
    }
  }

  public static final class ArmConstants {
    public static final int leftArmMotorID = 5;
    public static final int rightArmMotorID = 15;
    public static final int encoderID = 25;
    public static final double raiseArmSpeed = 1;
    public static final double lowerArmSpeed = -1;

    public static final class PIDValues {
      public static final double p = 4;
      public static final double i = 0;
      public static final double d = 1.7;
    }

    public static final class FeedForwardValues {
      public static final double kS = 0.07;
      public static final double kG = 0.4;
      public static final double kV = 3;
    }

    public static final class ArmPositions {
      public static final double lower = -0.11;
      public static final double lowerRad = lower;
      public static final double upper = 0.25;
      public static final double upperRad = upper * 2 * Math.PI;

      public static final double podium = 0.27;
    }
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 7;
    public static final int kickupMotorID = 17;
    public static final double intakeSpeed = 0.7;
    public static final double kickupSpeed = 0.7;

    public static final int currentLimit = 30;
  }

  public static final class ClimberConstants {
    public static final int climberMotorID = 20;
  }

  public static final class ShooterConstants {
    public static final int topShootMotorID = 16;
    public static final int bottomShootMotorID = 6;
    public static final double shooterSpeed = -1;

    public static final double targetFlywheelVelocity = 3700;
    public static final int currentLimit = 50;
  }

  public static final class AutoConstants {
    public static final class XPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class YPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RPID {
      public static final double p = 0.0015;
      public static final double i = 0;
      public static final double d = 0.0002;
    }

    public static final int medianFilter = 5;
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RotationPID {
      public static final double p = 6;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CANdleConstants {
    public static final int id = 50;
    public static final int ledCount = 50;
  }
}
