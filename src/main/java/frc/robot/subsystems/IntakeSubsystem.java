// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(IntakeConstants.intakeReversed);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
  }

  public void runIntake(double intakeMotorSpeed) {
    intakeMotor.set(intakeMotorSpeed);
  }

  public double getIntakeOutput() {
    return intakeMotor.getAppliedOutput();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
