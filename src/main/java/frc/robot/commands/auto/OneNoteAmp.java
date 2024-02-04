// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.autoCommands.AutoArm;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.commands.autoCommands.TriPIDDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAmp extends SequentialCommandGroup {
  private double xTarget;
  private double rotation;

  /** Creates a new OneNoteAmp. */
  public OneNoteAmp(Drivebase drivebase, AHRS gyro, Intake intake, Shooter shooter, Arm arm, boolean onBlueAlliance, DoubleSupplier xPose, DoubleSupplier yPose, DoubleSupplier angle) {
    if(onBlueAlliance) {
      xTarget = Units.inchesToMeters(72.5); //x target if on blue alliance
      rotation = 0.05; //spin at the beginning swaps direction based on alliance
    }
    else {
      xTarget = Units.inchesToMeters(578.77); //x target if on red alliance
      rotation = -0.05;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //TODO determine yTarget
    addCommands(
      new TimeDrive(drivebase, gyro, 0.5, rotation, 1), //drive forwards to get exit points, and rotate to see AprilTag
      new TriPIDDrive(drivebase, intake, false, xTarget, 0, -90, xPose, yPose, angle), //target and drive towards the amp
      new AutoArm(arm, ArmConstants.raiseArmSpeed, 0.5), //raise arm
      new AutoShoot(shooter, intake), //score preload in amp
      new AutoArm(arm, ArmConstants.lowerArmSpeed, 0.5) //lower arm

      );
  }
}
