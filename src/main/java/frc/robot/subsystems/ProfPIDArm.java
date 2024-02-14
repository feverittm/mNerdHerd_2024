// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.*;

public class ProfPIDArm extends ProfiledPIDSubsystem {

  private CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);
  private CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
  private CANcoder encoder = new CANcoder(ArmConstants.encoderID);
  private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS, FeedForwardValues.kG, FeedForwardValues.kV);
  double ffOutput;

  /** Creates a new ProfPIDArm. */
  public ProfPIDArm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            PIDValues.p,
            PIDValues.i,
            PIDValues.d,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffOutput = feedforward.calculate(setpoint.position, setpoint.velocity);
    leftArmMotor.setVoltage(output + ffOutput);
    rightArmMotor.setVoltage(output + ffOutput);
    // SmartDashboard.putNumber("Output", output);
    // SmartDashboard.putNumber("Setpoint", setpoint.position);
    // SmartDashboard.putNumber("Fed Firward", feedforward.calculate(setpoint.position, setpoint.velocity));
    // SmartDashboard.putNumberArray("PID, FF, and Combo", new double[]{output, ffOutput, output + ffOutput});
    // SmartDashboard.putNumber("Setpoint Velocity", setpoint.velocity);
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return getEncoder()*2*Math.PI;
  }

  public void disablePID() {
    disable();
  }
  
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderRadians();
  }
}
