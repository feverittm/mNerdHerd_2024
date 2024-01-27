// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;

public class TriPIDDrive extends Command {
  private Drivebase drivebase;
  private AHRS gyro;
  private PIDController xPID;
  private PIDController yPID;
  private PIDController rPID;
  DoubleSupplier xPose;
  DoubleSupplier yPose;
  DoubleSupplier angle;
  double xTarget;
  double yTarget;
  double rTarget;

  /** Creates a new TriPIDDrive. */
  public TriPIDDrive(Drivebase drivebase, AHRS gyro, double xTarget, double yTarget, double rTarget, DoubleSupplier xPose, DoubleSupplier yPose, DoubleSupplier angle) {
    this.drivebase = drivebase;
    this.gyro = gyro;
    this.xTarget = xTarget;
    this.yTarget = yTarget;
    this.rTarget = rTarget;
    this.xPose = xPose;
    this.yPose = yPose;
    this.angle = angle;
    this.xPID = new PIDController(AutoConstants.xPID.p, AutoConstants.xPID.i, AutoConstants.xPID.d);
    this.yPID = new PIDController(AutoConstants.yPID.p, AutoConstants.yPID.i, AutoConstants.yPID.d);
    this.rPID = new PIDController(AutoConstants.rPID.p, AutoConstants.rPID.i, AutoConstants.rPID.d);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.fieldOrientedDrive(
      xPID.calculate(xPose.getAsDouble(), xTarget),
      yPID.calculate(yPose.getAsDouble(), yTarget),
      rPID.calculate(angle.getAsDouble(), rTarget),
      -gyro.getYaw());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.fieldOrientedDrive(0, 0, 0, -gyro.getYaw());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
