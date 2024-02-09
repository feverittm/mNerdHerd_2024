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
public class TwoNoteAmp extends SequentialCommandGroup {
  private double xTargetAmp;
  private double noteTwoX;
  private double noteTwoAngle;
  private double rotation;

  /** Creates a new OneNoteAmp. */
  public TwoNoteAmp(Drivebase drivebase, AHRS gyro, Intake intake, Shooter shooter, Arm arm, 
    boolean onBlueAlliance, DoubleSupplier xPose, DoubleSupplier yPose, DoubleSupplier angle) {
    if(onBlueAlliance) {
      xTargetAmp = Units.inchesToMeters(72.5); //x target if on blue alliance
      rotation = 0.05; //spin at the beginning swaps direction based on alliance
      noteTwoAngle = -45; //angle when getting the second note
      noteTwoX = 0; //TODO X pose of second note
    }
    else {
      xTargetAmp = Units.inchesToMeters(578.77); //x target if on red alliance
      rotation = -0.05;
      noteTwoAngle = -135;
      noteTwoX = 0;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //TODO determine yTargets
    addCommands(
      new TimeDrive(drivebase, gyro, 0, rotation, 0.7), //rotate to see AprilTag
      new TriPIDDrive(drivebase, intake, false, xTargetAmp, 0, -90, xPose, yPose, angle), //target and drive towards the amp
      new AutoArm(arm, ArmConstants.raiseArmSpeed, 0.5), //raise arm
      new AutoShoot(shooter, intake), //score preload in amp
      new AutoArm(arm, ArmConstants.lowerArmSpeed, 0.5), //lower arm
      new TriPIDDrive(drivebase, intake, true, noteTwoX, 0, noteTwoAngle, xPose, yPose, angle), //pick up second note
      new TriPIDDrive(drivebase, intake, false, xTargetAmp, 0, -90, xPose, yPose, angle), //return to amp
      new AutoArm(arm, ArmConstants.raiseArmSpeed, 0.5), //raise arm
      new AutoShoot(shooter, intake), //score second note in amp
      new AutoArm(arm, ArmConstants.lowerArmSpeed, 0.5) //lower arm
      );
  }
}
