// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexNote extends Command {
  /** Creates a new RunIntake. */
  public final IntakeSubsystem intake;
  public final IndexSubsystem index;

  public IndexNote(IntakeSubsystem intake, IndexSubsystem index) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.index = index;

    addRequirements(this.intake, this.index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    index.s
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
