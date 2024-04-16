// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftShootMotor = new CANSparkMax(ShooterConstants.leftShootMotorID, MotorType.kBrushless);
  private CANSparkMax rightShootMotor = new CANSparkMax(ShooterConstants.rightShootMotorID, MotorType.kBrushless);

  private RelativeEncoder leftShootEncoder = leftShootMotor.getEncoder();

  /** Creates a new Shooter. */
  public Shooter() {
    leftShootMotor.restoreFactoryDefaults();
    rightShootMotor.restoreFactoryDefaults();

    rightShootMotor.setInverted(true);

    leftShootMotor.setIdleMode(IdleMode.kBrake);
    rightShootMotor.setIdleMode(IdleMode.kBrake);

    leftShootMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);
    rightShootMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);

    rightShootMotor.follow(leftShootMotor);

    leftShootEncoder = leftShootMotor.getEncoder();
  }

  public void spinShooter(double speed) {
    leftShootMotor.set(speed);
  }

  public boolean isReady() {
    return leftShootEncoder.getVelocity() > ShooterConstants.targetFlywheelVelocity;
  }

  public void stopShooter() {
    leftShootMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Flywheel vel", -topShootEncoder.getVelocity());
  }
}
