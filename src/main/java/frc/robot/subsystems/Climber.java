// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax leftClimberMotor = new CANSparkMax(ClimberConstants.leftClimberMotorId, MotorType.kBrushless);
  private CANSparkMax rightClimberMotor = new CANSparkMax(ClimberConstants.rightClimberMotorId, MotorType.kBrushless);
  private DigitalInput leftBottomLimit = new DigitalInput(ClimberConstants.leftClimberSensorId);
  private RelativeEncoder leftClimberEncoder;


  public Climber() {
    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    leftClimberMotor.setInverted(false);
    rightClimberMotor.setInverted(true);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    rightClimberMotor.follow(leftClimberMotor);

    leftClimberEncoder = leftClimberMotor.getEncoder();
  }

  public double getClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public boolean atBottom() {
    return(leftBottomLimit.get());
  }

  public void runClimber(double motorSpeed) {
    leftClimberMotor.set(motorSpeed);
  }

  public void stopClimber() {
    leftClimberMotor.stopMotor();
  }

  public void setBrakeMode() {
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    leftClimberMotor.setIdleMode(IdleMode.kCoast);
    rightClimberMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
