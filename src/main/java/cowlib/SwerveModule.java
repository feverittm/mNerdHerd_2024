// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController pidController;
  private CANcoder encoder;
  private boolean inverted;
  private double offset;
  private double maxVelocity;
  private double maxVoltage;

  public SwerveModule(int angleMotor, int speedMotor, int encoder, boolean drive_inverted, double offset,
      double maxVelocity, double maxVoltage) {
    this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
    this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
    this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
    this.encoder = new CANcoder(encoder);
    this.inverted = drive_inverted;
    this.offset = offset;
    this.maxVelocity = maxVelocity;
    this.maxVoltage = maxVoltage;

    this.pidController.enableContinuousInput(-180, 180);
  }

  public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
    this(config.angleMotorId,
        config.driveMotorId,
        config.encoderId,
        config.drive_inverted,
        config.offset,
        maxVelocity,
        maxVoltage);

    angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
  }

  private void drive(double speedMetersPerSecond, double angle) {
    double voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
    speedMotor.setVoltage(voltage * (this.inverted ? -1 : 1));
    angleMotor.setVoltage(-pidController.calculate(this.getEncoder(), angle));
  }

  public void drive(SwerveModuleState state) {
    SwerveModuleState optimized = SwerveModuleState.optimize(state, new Rotation2d(getEncoderRadians()));
    this.drive(optimized.speedMetersPerSecond, optimized.angle.getDegrees());
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return Units.degreesToRadians(encoder.getAbsolutePosition().getValueAsDouble());
  }
}
