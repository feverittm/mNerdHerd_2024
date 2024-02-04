// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topShootMotor = new CANSparkMax(ShooterConstants.topShootMotorID, MotorType.kBrushless);
  private CANSparkMax bottomShootMotor = new CANSparkMax(ShooterConstants.bottomShootMotorID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    topShootMotor.restoreFactoryDefaults();
    bottomShootMotor.restoreFactoryDefaults();

    topShootMotor.setIdleMode(IdleMode.kCoast);
    bottomShootMotor.setIdleMode(IdleMode.kCoast);

    bottomShootMotor.follow(topShootMotor);
  }

  public void spinShooter(double speed) {
    topShootMotor.set(speed);
  }

  public void stopShooter() {
    topShootMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
