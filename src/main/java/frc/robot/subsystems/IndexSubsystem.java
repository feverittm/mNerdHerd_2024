// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */
  private CANSparkMax indexMotor = new CANSparkMax(IndexerConstants.indexMotorId, MotorType.kBrushless);
  private RelativeEncoder indexEncoder;
  public DigitalInput noteSensor = new DigitalInput(IndexerConstants.noteSensorid);

  public Trigger noteTrigger = new Trigger(noteSensor::get);
  public static boolean noteLatch = false;

  public IndexSubsystem() {
    indexMotor.restoreFactoryDefaults();
    indexMotor.setIdleMode(IdleMode.kBrake);
    indexMotor.setInverted(IntakeConstants.intakeReversed);
    indexMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
    indexEncoder = indexMotor.getEncoder();
    indexEncoder.setPosition(0);

    noteLatch = false;
    noteTrigger.onTrue(setNoteLatchCommand());
  }

  public void runIndex(double speed) {
    indexMotor.set(speed);
  }

  public double getIndexPosition() {
    return indexEncoder.getPosition();
  }

  public Command setNoteLatchCommand() {
    return this.runOnce(() -> noteLatch = true );
  }

  public Command resetNoteLatchCommand() {
    return this.runOnce(() -> noteLatch = false );
  }

  public boolean getNoteLatch() {
    return noteLatch;
  }

  public boolean getNoteStatus() {
    return !noteSensor.get();
  }

  public void stopIndex() {
    indexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
