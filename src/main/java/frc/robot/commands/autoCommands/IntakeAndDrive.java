// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;

public class IntakeAndDrive extends Command {
  private final Drivebase drivebase;
  private final Intake intake;
  private double speed;
  private double startTime;
  private double delay;

  /** Creates a new IntakeDrive. */
  public IntakeAndDrive(Drivebase drivebase, Intake intake, double speed, double delay) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.speed = speed;
    this.delay = delay;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase, this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed);
    drivebase.robotOrientedDrive(speed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    drivebase.robotOrientedDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > delay;
  }
}
