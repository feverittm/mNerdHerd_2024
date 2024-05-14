// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSparkMax m_leftClimberMotor;
  private CANSparkMax m_rightClimberMotor;
  private RelativeEncoder m_leftClimberEncoder;
  private DigitalInput m_leftClimberLimitSwitch;
  private DigitalInput m_rightClimberLimitSwitch;

  public ClimberSubsystem() {
    m_leftClimberMotor = new CANSparkMax(ClimberConstants.leftClimberMotorId, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.leftClimberMotorId, MotorType.kBrushless);

    m_leftClimberMotor.restoreFactoryDefaults();
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    m_leftClimberMotor.setInverted(ClimberConstants.leftClimberInverted);

    m_rightClimberMotor.restoreFactoryDefaults();
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
    m_leftClimberMotor.setInverted(ClimberConstants.rightClimberInverted);

    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_leftClimberEncoder.setPosition(0);

    m_leftClimberLimitSwitch = new DigitalInput(ClimberConstants.leftClimberSensorId);
    m_rightClimberLimitSwitch = new DigitalInput(ClimberConstants.rightClimberSensorId);
  }

  public boolean getClimberStatus() {
    return m_leftClimberLimitSwitch.get() || m_rightClimberLimitSwitch.get();
  }

  public double getClimberPosition() {
    return m_leftClimberEncoder.getPosition();
  }

  public void stopClimber() {
    m_leftClimberMotor.set(0);
    m_rightClimberMotor.set(0);
  }

  public void runClimber(double leftSpeed, double rightSpeed) {
    m_leftClimberMotor.set(leftSpeed);
    m_rightClimberMotor.set(rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
