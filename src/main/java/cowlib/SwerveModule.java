// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private RelativeEncoder speedEncoder;
  private PIDController pidController;
  private SparkAbsoluteEncoder encoder;
  private double maxVelocity;
  private double maxVoltage;

  public SwerveModule(int angleMotorId, int speedMotorId, boolean driveMotorReversed, boolean angleMotorReversed,
      boolean angleEncoderReversed, double angleEncoderConversionFactor, double angleEncoderOffset,
      double maxVelocity, double maxVoltage) {
    this.angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
    this.speedMotor = new CANSparkMax(speedMotorId, MotorType.kBrushless);

    this.angleMotor.restoreFactoryDefaults();
    this.speedMotor.restoreFactoryDefaults();

    this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
    this.encoder = this.angleMotor.getAbsoluteEncoder();
    this.maxVelocity = maxVelocity;
    this.maxVoltage = maxVoltage;

    this.pidController.enableContinuousInput(-180, 180);

    this.speedMotor.setInverted(driveMotorReversed);
    this.angleMotor.setInverted(angleMotorReversed);

    this.speedMotor.setIdleMode(IdleMode.kBrake);
    this.angleMotor.setIdleMode(IdleMode.kCoast);

    // Set scaling factors
    this.speedEncoder = this.speedMotor.getEncoder();
    double driveReduction = 1.0 / 6.75;
    double WHEEL_DIAMETER = 0.1016;
    double rotationsToDistance = driveReduction * WHEEL_DIAMETER * Math.PI;

    this.encoder.setZeroOffset(angleEncoderOffset);
    this.encoder.setPositionConversionFactor(1);
    this.encoder.setVelocityConversionFactor(1);
    this.speedEncoder.setPositionConversionFactor(rotationsToDistance);
    this.speedEncoder.setVelocityConversionFactor(rotationsToDistance / 60);

    // Try and 'tweak' the Rotation2d model to get it to read the correct angle.
    Rotation2d dummy_rotation = getRotation();
  }

  public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
    this(config.angleMotorId,
        config.driveMotorId,
        config.driveMotorReversed,
        config.angleMotorReversed,
        config.angleEncoderReversed,
        config.angleEncoderConversionFactor,
        config.angleEncoderOffset,
        maxVelocity,
        maxVoltage);

    angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
  }

  /**
   * drive:
   * 
   * @param speedMetersPerSecond
   * @param angle
   *                             Basic drive code. Lots of debug information sent
   *                             to the dashboard so that
   *                             we can watch what is happening. Remember that
   *                             everything should be CCW positive.
   *                             It can be easier to load the Shuffleboard
   *                             configuration called "Mofule debugging Dashboard"
   */
  private void drive(double speedMetersPerSecond, double angle) {
    double drive_voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
    double angle_voltage = pidController.calculate(this.getEncoder(), angle);

    if (angleMotor.getDeviceId() == 3) {
      SmartDashboard.putNumber("Drive/Module drive speed", speedMetersPerSecond);
      SmartDashboard.putNumber("Drive/Module drive angle", angle); // should be in degrees (mismatch????)
      SmartDashboard.putNumber("Debug/Module encoder angle", this.getEncoder()); // return in degrees
      SmartDashboard.putNumber("Debug/Drive V", drive_voltage);
      SmartDashboard.putNumber("Debug/Drive A", angle_voltage);
    }

    speedMotor.setVoltage(drive_voltage);
    angleMotor.setVoltage(angle_voltage);

  }

  /**
   * drive:
   * @param state of the module (velocity and angle)
   */
  public void drive(SwerveModuleState state) {
    double rot = getRotation().getRadians();
    SwerveModuleState optimized = SwerveModuleState.optimize(state, getRotation());

    if (angleMotor.getDeviceId() == 3) {
      SmartDashboard.putNumber("Debug/Module speed", state.speedMetersPerSecond);
      SmartDashboard.putNumber("Debug/Module angle", state.angle.getDegrees());
      SmartDashboard.putNumber("Debug/Optimize Encoder", rot);
    }

    // a little wierd logic. Call the other drive code above to actually move the
    // module.
    this.drive(optimized.speedMetersPerSecond, optimized.angle.getDegrees());
  }

  /**
   * getEncoder:
   * 
   * @return Return the module angle in degrees 0-360. CCW positive.
   *         Straight Forward should be 0 (with the addjustment of module offset)
   */
  public double getEncoder() {
    return encoder.getPosition() * 360.0;
  }

  /*
   * Return the applied voltage on the drive motor (0-12V)
   */
  public double getDriveOutput() {
    return speedMotor.getAppliedOutput();
  }

  /*
   * Return a rotation object for the module absolute encoder.
   */
  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getEncoder());
  }

  /*
   * Return the absolute encoder position in radians (0-2pi)
   */
  public double getEncoderRadians() {
    return Units.degreesToRadians(getEncoder());
  }

  /*
   * What is the position of the module using the encoder information.
   * The encoder cinfiguration should be set so that this function will
   * return the valid location in meters.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(speedEncoder.getPosition(), getRotation());
  }

  /*
   * Another view of the module state, showing velocity instead of position
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(speedEncoder.getVelocity(), getRotation());
  }
}
