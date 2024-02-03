// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class Drive extends Command {

  private final Drivebase drivebase;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier rot;
  private final AHRS gyro;

  /** Creates a new Drive. */
  public Drive(Drivebase drivebase, AHRS gyro, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drivebase = drivebase;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    this.gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var x = speedX.getAsDouble();
    var y = speedY.getAsDouble();
    var r = rot.getAsDouble();

    // drivebase.robotOrientedDrive(x, y, r);
    drivebase.fieldOrientedDrive(x, y, r, -gyro.getYaw());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
