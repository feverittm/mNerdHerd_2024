// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.autoCommands.TestTriPID;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.subsystems.Drivebase;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Drivebase drivebase = new Drivebase();

  private final AHRS gyro = new AHRS();

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private MedianFilter xFilter = new MedianFilter(AutoConstants.medianFilter);
  private MedianFilter yFilter = new MedianFilter(AutoConstants.medianFilter);
  private MedianFilter angleFilter = new MedianFilter(AutoConstants.medianFilter);
  private final DoubleSupplier filteredXPose = 
    () -> xFilter.calculate(
      Math.abs(limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[0])); //TODO make sure abs doesn't screw things up

    private final DoubleSupplier filteredYPose = 
    () -> yFilter.calculate(
      limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[1]); //TODO, make sure these are the right values for TY and RZ

    private final DoubleSupplier filteredAnlge = 
    () -> angleFilter.calculate(
      limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[5]);

  private final TimeDrive timeDrive = new TimeDrive(drivebase, 0.2, 5);
  private final TestTriPID testAuto = new TestTriPID(drivebase, gyro, filteredXPose, filteredYPose, filteredAnlge);

  private static CommandXboxController driveStick = new CommandXboxController(0);

  SendableChooser<Command> commandChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    commandChooser.addOption("Timed drive", timeDrive);
    commandChooser.addOption("AprilTag Auto Test", testAuto);
    SmartDashboard.putData(commandChooser);

    // Configure the trigger bindings
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            gyro,
            () -> scaleTranslationAxis(driveStick.getLeftY()*0.8),
            () -> scaleTranslationAxis(driveStick.getLeftX()*0.8),
            () -> scaleRotationAxis(driveStick.getRightX())*0.8));

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

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  private double scaleTranslationAxis(double input) {
    return deadband(-squared(input), DriveConstants.deadband) * drivebase.getMaxVelocity();
  }

  private double scaleRotationAxis(double input) {
    return deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngleVelocity();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public Double[] getBotposeDoubles() {
    return new Double[]{filteredXPose.getAsDouble(), filteredYPose.getAsDouble(), filteredAnlge.getAsDouble()};
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
    driveStick.pov(0).onTrue(new InstantCommand(gyro::reset));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return commandChooser.getSelected();
  }
}
