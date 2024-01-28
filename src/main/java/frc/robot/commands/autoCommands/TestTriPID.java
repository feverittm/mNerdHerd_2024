// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTriPID extends SequentialCommandGroup {

  /** Creates a new TestTriPID. */
  public TestTriPID(Drivebase drivebase, AHRS gyro, DoubleSupplier xPose, DoubleSupplier yPose, DoubleSupplier angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TriPIDDrive(drivebase, gyro, 5, 1.3, 0, xPose, yPose, angle)
    );
  }
}
