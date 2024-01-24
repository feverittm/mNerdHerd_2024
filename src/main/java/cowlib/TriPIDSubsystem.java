// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

/**
 * A subsystem that uses a {@link PIDController} to control an output. The controller is run
 * synchronously from the subsystem's periodic() method.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class TriPIDSubsystem extends SubsystemBase {
    /*
     * enum used throughout the TriPIDSubsystem class to select which controller to use
     */
    public enum Controller {
        A,
        B,
        C
    }

  protected final PIDController m_controllerA;
  protected final PIDController m_controllerB;
  protected final PIDController m_controllerC;
  protected boolean m_enabled;

  /**
   * Creates a new PIDSubsystem.
   *
   * @param controller the PIDController to use
   * @param initialPosition the initial setpoint of the subsystem
   */
  public TriPIDSubsystem(PIDController controllerA, PIDController controllerB, PIDController controllerC, double initialPositionA, double initialPositionB, double initialPositionC) {
    m_controllerA = requireNonNullParam(controllerA, "controllerA", "PIDSubsystem");
    m_controllerB = requireNonNullParam(controllerB, "controllerB", "PIDSubsystem");
    m_controllerC = requireNonNullParam(controllerC, "controllerC", "PIDSubsystem");
    setSetpoint(initialPositionA, initialPositionB, initialPositionC);
    addChild("PID ControllerA", m_controllerA);
    addChild("PID ControllerB", m_controllerB);
    addChild("PID ControllerC", m_controllerC);

  }

  /**
   * Creates a new PIDSubsystem. Initial setpoint is zero.
   *
   * @param controller the PIDController to use
   */
  public TriPIDSubsystem(PIDController controllerA, PIDController controllerB, PIDController controllerC) {
    this(controllerA, controllerB, controllerC, 0, 0, 0);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controllerA.calculate(getMeasurement()[0]), m_controllerA.getSetpoint(),
      m_controllerB.calculate(getMeasurement()[1]), m_controllerB.getSetpoint(),
      m_controllerC.calculate(getMeasurement()[2]), m_controllerC.getSetpoint());
    }
  }

  public PIDController[] getControllers() {
    return new PIDController[] { m_controllerA, m_controllerB, m_controllerC };
  }

  public PIDController getController(Controller controller) {
    switch(controller) {
        case A: return m_controllerA;
        case B: return m_controllerB;
        case C: return m_controllerC;

        default: return null;
        }
    }

  /**
   * Sets the setpoint for the subsystem.
   *
   * @param setpoint the setpoint for the subsystem
   */
  public void setSetpoint(double setpointA, double setpointB, double setpointC) {
    m_controllerA.setSetpoint(setpointA);
    m_controllerB.setSetpoint(setpointB);
    m_controllerC.setSetpoint(setpointC);
  }

  /**
   * Returns the current setpoint of the subsystem.
   *
   * @return The current setpoint
   */
  public double getSetpoint(Controller controller) {
    switch(controller) {
        case A: return m_controllerA.getSetpoint();
        case B: return m_controllerB.getSetpoint();
        case C: return m_controllerC.getSetpoint();
        default: return 0;
    }
  }

  /**
   * Uses the output from the PIDController.
   *
   * @param output the output of the PIDController
   * @param setpoint the setpoint of the PIDController (for feedforward)
   */
  protected abstract void useOutput(double outputA, double outputB, double outputC, double setpointA, double setpointB, double setpointC);

  /**
   * Returns the measurement of the process variable used by the PIDController.
   *
   * @return the measurement of the process variable
   */
  protected abstract double[] getMeasurement();

  /** Enables the PID control. Resets the controller. */
  public void enable() {
    m_enabled = true;
    m_controllerA.reset();
    m_controllerB.reset();
    m_controllerC.reset();
  }

  /** Disables the PID control. Sets output to zero. */
  public void disable() {
    m_enabled = false;
    useOutput(0, 0, 0, 0, 0, 0);
  }

  /**
   * Returns whether the controller is enabled.
   *
   * @return Whether the controller is enabled.
   */
  public boolean isEnabled() {
    return m_enabled;
  }
}