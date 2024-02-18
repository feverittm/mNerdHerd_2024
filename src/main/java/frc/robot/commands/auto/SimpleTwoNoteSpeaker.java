// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.autoCommands.IntakeAndDrive;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleTwoNoteSpeaker extends SequentialCommandGroup {
  /** Creates a new SimpleTwoNoteSpeaker. */
  public SimpleTwoNoteSpeaker(Drivebase drivebase, AHRS gyro, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(shooter, intake), //shoot preloaded note
      new IntakeAndDrive(drivebase, intake, 0.75, 2.5), //drive forward with the intake on
      new WaitCommand(0.75),
      new TimeDrive(drivebase, gyro, -0.75, 0, 2.75), //drive back after hopefully picking up field note
      new AutoShoot(shooter, intake) //shoot note
    );
  }
}
