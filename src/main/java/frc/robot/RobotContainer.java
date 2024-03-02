// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.Rumble;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.armCommands.MoveArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AHRS gyro = new AHRS();
  private final DigitalInput beamBreak = new DigitalInput(0);

  private final Drivebase drivebase = new Drivebase(gyro);
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private static XboxController driveStick = new XboxController(0);

  private SendableChooser<Command> autoChooser;

  private double mapped = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    var shootComp = Commands.race(new Shoot(shooter, ShooterConstants.shooterSpeed),
        Commands.sequence(Commands.waitSeconds(0.5),
            Commands.race(new RunIntake(intake, 0.3, IntakeConstants.kickupSpeed), Commands.waitSeconds(0.4))));

    // var armUp = Commands.race(
    //     new MoveArm(arm, () -> ArmConstants.raiseArmSpeed),
    //     Commands.waitSeconds(1));

    // var armDown = Commands.race(
    //     new MoveArm(arm, () -> ArmConstants.lowerArmSpeed),
    //     Commands.waitSeconds(1.5));

    var ampShoot = Commands.race(
        Commands.parallel(
            new Shoot(shooter, ShooterConstants.shooterSpeed),
            new RunIntake(intake, 0.3, IntakeConstants.kickupSpeed)),
        Commands.waitSeconds(0.4));

    NamedCommands.registerCommand("Intake",
        new RunIntake(intake, IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed));
    NamedCommands.registerCommand("Stop Intake",
        new RunIntake(intake, 0, 0));
    NamedCommands.registerCommand("Shoot", shootComp);
    // NamedCommands.registerCommand("Amp Score", Commands.sequence(armUp, ampShoot, armDown));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure the trigger bindings
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> scaleTranslationAxis(driveStick.getLeftY()),
            () -> scaleTranslationAxis(driveStick.getLeftX()),
            () -> scaleRotationAxis(driveStick.getRightX())));

    arm.setDefaultCommand(
        new MoveArm(
            arm,
            () -> getArmControl(driveStick.getRightTriggerAxis() - driveStick.getLeftTriggerAxis())));

    configureBindings();
  }

  /**
   * TODO: Investigate which has an applyDeadband function
   *
   * {@link edu.wpi.first.math.MathUtil}
   */
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double getArmControl(double trigger) {
    if (trigger > 0) {
      mapped = trigger * ArmConstants.raiseArmSpeed;
    } else if (trigger < 0) {
      mapped = -trigger * ArmConstants.lowerArmSpeed;
    } else {
      mapped = 0;
    }

    return mapped;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  private double cube(double input) {
    return Math.copySign(input * input * input, input);
  }

  private double scaleTranslationAxis(double input) {
    return deadband(-cube(input), DriveConstants.deadband) * drivebase.getMaxVelocity() * 0.8;
  }

  private double scaleRotationAxis(double input) {
    return deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngleVelocity() * -0.6;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Blue;
    }
    return false;
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new POVButton(driveStick, 0).onTrue(new InstantCommand(gyro::reset)); // resets the gyro for field oriented controll
    new JoystickButton(driveStick, Button.kStart.value)
        .whileTrue(new RunIntake(intake, -IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed)); // reverse intake
    new JoystickButton(driveStick, Button.kLeftBumper.value).whileTrue(Commands.parallel(
        new RunIntake(intake, IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed), // toggle intake on/off
        new Rumble(driveStick, beamBreak))); // rumble controller if note is visible
    new JoystickButton(driveStick, Button.kRightBumper.value)
        .whileTrue(new Shoot(shooter, ShooterConstants.shooterSpeed)); // spin up flywheels while button is held
    new JoystickButton(driveStick, Button.kRightBumper.value).onFalse( // shoot note when button is released
        Commands.race(
            Commands.parallel(
                new Shoot(shooter, ShooterConstants.shooterSpeed),
                new RunIntake(intake, 0.3, IntakeConstants.kickupSpeed)),
            new WaitCommand(0.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
