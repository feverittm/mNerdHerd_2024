// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.*;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  private final Arm arm;
  private final ProfiledPIDController pidController;
  private final ArmFeedforward feedforward;
  private double speed;
  private double angleGoal;
  private double pidOutput;
  private double ffOutput;

  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, double speed /*, double angleGoal*/) {
    this.arm = arm;
    this.speed = speed;
    // this.angleGoal = angleGoal;
    this.pidController = new ProfiledPIDController(PIDValues.p, PIDValues.i, PIDValues.d, 
      new TrapezoidProfile.Constraints(0, 0));
    this.feedforward = new ArmFeedforward(FeedForwardValues.kS, FeedForwardValues.kG, FeedForwardValues.kV);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pidOutput = pidController.calculate(arm.getEncoderRadians(), angleGoal);
    // ffOutput = feedforward.calculate(angleGoal, 0);
    // arm.setArmVoltage(pidOutput + ffOutput);
    arm.setArmSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
