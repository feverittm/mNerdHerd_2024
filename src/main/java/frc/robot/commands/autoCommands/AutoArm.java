// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.armCommands.MoveArm;
import frc.robot.subsystems.Arm;

public class AutoArm extends Command {
  private final MoveArm moveArm;
  private double startTime;
  private double delay;
  // private double speed;

  /** Creates a new AutoArm. */
  public AutoArm(Arm arm, double speed, double delay) {
    this.moveArm = new MoveArm(arm, () -> speed); //TODO this is weird but also it's unimportant for scrimmage but is important afterwards
    this.delay = delay;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveArm.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > delay;
  }
}
