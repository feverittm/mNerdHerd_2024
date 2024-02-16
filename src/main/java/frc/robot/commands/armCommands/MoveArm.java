// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  private final Arm arm;
  private DoubleSupplier speed;
  // private double speed;

  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmSpeed(speed.getAsDouble());
    //uncomment this to controll max up/down speed of arm
    // if(speed.getAsDouble() > 0) {
    //   arm.setArmSpeed(Math.min(speed.getAsDouble(), ArmConstants.raiseArmSpeed));
    // }
    // else {
    //   arm.setArmSpeed(Math.max(speed.getAsDouble(), ArmConstants.lowerArmSpeed));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*we don't want to stop the motor at the end of the command because we want to be constantly running the motor in the desired
    direction. The limit sitches will stop the arm at either end*/
    //this line is for when the limit switches aren't attached
    arm.setArmSpeed(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
