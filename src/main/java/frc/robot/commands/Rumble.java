// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;

public class Rumble extends Command {
  private final XboxController driveStick;
  private final DigitalInput beamBreak;
  private final CANdleSystem candle;
  private final BooleanSupplier shooterReady;

  /** Creates a new Rumble. */
  public Rumble(XboxController driveStick, DigitalInput beamBreak, CANdleSystem candle, BooleanSupplier shooterReady) {
    this.driveStick = driveStick;
    this.beamBreak = beamBreak;
    this.candle = candle;
    this.shooterReady = shooterReady;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.get()) {
      driveStick.setRumble(RumbleType.kBothRumble, 0);
      candle.setBlue();
    } else if (shooterReady.getAsBoolean()) {
      driveStick.setRumble(RumbleType.kBothRumble, 0);
      candle.setGreen();
    } else {
      driveStick.setRumble(RumbleType.kBothRumble, 1);
      candle.setOrange();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveStick.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
