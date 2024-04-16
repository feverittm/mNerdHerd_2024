// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Rumble;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.SignalLogger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();

  private static XboxController driveStick = new XboxController(0);

  // private static CommandXboxController c_driveStick2 = new
  // CommandXboxController(1);
  private static CommandXboxController c_driveStick = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);

    var shootComp = Commands.race(new Shoot(shooter, -0.95),
        Commands.sequence(Commands.waitSeconds(0.55),
            Commands.race(new RunIntake(intake, 0.5, IntakeConstants.kickupSpeed), Commands.waitSeconds(0.25))));

    var defenceShoot = Commands.parallel(
        new Shoot(shooter, -0.15),
        new RunIntake(intake, 0.5, IntakeConstants.kickupSpeed));

    var stopDefence = Commands.parallel(
        new Shoot(shooter, 0),
        new RunIntake(intake, 0, 0));

    NamedCommands.registerCommand("Intake",
        new RunIntake(intake, IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed));
    NamedCommands.registerCommand("Stop Intake",
        new RunIntake(intake, 0, 0));
    NamedCommands.registerCommand("Shoot", shootComp);
    NamedCommands.registerCommand("Defence Shoot", defenceShoot);
    NamedCommands.registerCommand("Stop Defence Shoot", stopDefence);

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure the trigger bindings
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> getScaledXY(),
            () -> scaleRotationAxis(driveStick.getRightX())));

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

  private double[] getXY() {
    double[] xy = new double[2];
    xy[0] = deadband(driveStick.getLeftX(), DriveConstants.deadband);
    xy[1] = deadband(driveStick.getLeftY(), DriveConstants.deadband);
    return xy;
  }

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Convert to Polar coordinates
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[1], xy[0]);

    // Square radius and scale by max velocity
    r = r * r * drivebase.getMaxVelocity();

    // Convert to Cartesian coordinates
    xy[0] = r * Math.cos(theta);
    xy[1] = r * Math.sin(theta);

    return xy;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  @SuppressWarnings("unused")
  private double cube(double input) {
    return Math.copySign(input * input * input, input);
  }

  @SuppressWarnings("unused")
  private double scaleTranslationAxis(double input) {
    return deadband(-squared(input), DriveConstants.deadband) * drivebase.getMaxVelocity();
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
    // Gyro Reset
    c_driveStick.povUp().onTrue(Commands.runOnce(gyro::reset));

    // Intake
    c_driveStick.leftBumper().whileTrue(Commands.parallel(
        new RunIntake(intake, IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed), // toggle intake on/off
        new Rumble(driveStick, beamBreak, () -> false))); // rumble controller if note is visible

    // Charge Shooter
    c_driveStick.rightBumper()
        .whileTrue(Commands.parallel(
            new Shoot(shooter, ShooterConstants.shooterSpeed),
            new RunIntake(intake, 0.5, -IntakeConstants.kickupSpeed),
            new Rumble(driveStick, beamBreak, shooter::isReady))); // spin up flywheels while button is held

    // Release Shooter
    c_driveStick.rightBumper().onFalse( // shoot note when button is released
        Commands.sequence(
            Commands.race(
                Commands.parallel(
                    new Shoot(shooter, ShooterConstants.shooterSpeed),
                    new RunIntake(intake, 0.5, IntakeConstants.kickupSpeed),
                    new Rumble(driveStick, beamBreak, shooter::isReady)),
                new WaitCommand(0.5))));

    // Spit out note
    c_driveStick.start()
        .whileTrue(new RunIntake(intake, -IntakeConstants.intakeSpeed, -IntakeConstants.kickupSpeed));

    // Driver climb controls
    c_driveStick.x().whileTrue(new Climb(climber, 1)); // climber up
    c_driveStick.b().whileTrue(new Climb(climber, -1)); // climber down

    // Codriver climb controls
    // c_driveStick2.y().whileTrue(new Climb(climber, 1));
    // c_driveStick2.a().whileTrue(new Climb(climber, -1));
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
