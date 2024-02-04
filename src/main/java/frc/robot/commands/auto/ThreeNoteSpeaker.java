// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.autoCommands.TriPIDDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteSpeaker extends SequentialCommandGroup {
  private double noteThreeX;
  private double noteThreeAngle;
  private double speakerX;
  private double speakerAngle;
  /** Creates a new ThreeNoteSpeaker. */
  public ThreeNoteSpeaker(Drivebase drivebase, AHRS gyro, Intake intake, Shooter shooter, boolean onBlueAlliance, DoubleSupplier xPose, DoubleSupplier yPose, DoubleSupplier angle) {
    if(onBlueAlliance) {
      noteThreeX = 0;
      noteThreeAngle = 0; //this one will depend on which note we're going for
      speakerX = 0;
      speakerAngle = 0; //this one is supposed to be 0. the others are placeholders
    }
    else {
      noteThreeX = 0;
      noteThreeAngle = 0;
      speakerX = 0;
      speakerAngle = -180;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SimpleTwoNoteSpeaker(drivebase, gyro, intake, shooter), //collect and score the first two notes
      new TriPIDDrive(drivebase, intake, true, noteThreeX, 0, noteThreeAngle, xPose, yPose, angle), //pick up third note
      new TriPIDDrive(drivebase, intake, false, speakerX, Units.inchesToMeters(218.42), speakerAngle, xPose, yPose, angle), //return to speaker
      new AutoShoot(shooter, intake) //score third note
    );
  }
}
