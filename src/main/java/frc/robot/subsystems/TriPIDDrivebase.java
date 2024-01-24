// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cowlib.TriPIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class TriPIDDrivebase extends TriPIDSubsystem {
  private final Drivebase drivebase;

  /** Creates a new TestPIDSubsystem. */
  public TriPIDDrivebase(Drivebase drivebase) {
    super(
        // The PIDControllers used by the subsystem
        new PIDController(0, 0, 0), // x-translation
        new PIDController(0, 0, 0), // y-translation
        new PIDController(0, 0, 0)); // rotation

    this.drivebase = drivebase;
  }

  @Override
  public void useOutput(double outputA, double outputB, double outputC, double setpointA, double setpointB,
      double setpointC) {
    // Use the output here
    drivebase.robotOrientedDrive(outputA, outputB, outputC);
  }

  @Override
  public double[] getMeasurement() {
    // Return the process variable measurements here
    // insert AprilTag values here
    return new double[] { 0, 0, 0 };
  }
}
