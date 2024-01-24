// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TimeDrive extends CommandBase {
  private final Drivebase drivebase;
  private final double speed;
  private final double delay;
  private double startTime = 0;

  /** Creates a new TimeDrive. */
  public TimeDrive(Drivebase drivebase, double speed, double delay) {
    this.drivebase = drivebase;
    this.speed = speed;
    this.delay = delay;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.robotOrientedDrive(0, speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.robotOrientedDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > delay;
  }
}
