package org.frc3512.robot.subsystems.hopper.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Roller motors
    public boolean roller1Connected = false;
    public double roller1VelocityRPS = 0.0;
    public double roller1AppliedVolts = 0.0;
    public double roller1CurrentAmps = 0.0;
    public double roller1TempCelsius = 0.0;

    public boolean roller2Connected = false;
    public double roller2VelocityRPS = 0.0;
    public double roller2AppliedVolts = 0.0;
    public double roller2CurrentAmps = 0.0;
    public double roller2TempCelsius = 0.0;

    // Extension motor
    public boolean extensionConnected = false;
    public double extensionPositionInches = 0.0;
    public double extensionVelocityInchesPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;
    public double extensionTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run roller motors at the specified velocity (mechanism RPM). */
  public default void setRollerVelocity(double mechanismRPM) {}

  /** Run roller motors at the specified open loop value. */
  public default void setRollerOpenLoop(double output) {}

  /** Run extension motor to the specified position (inches). */
  public default void setExtensionPosition(double positionInches) {}

  /** Run extension motor at the specified open loop value. */
  public default void setExtensionOpenLoop(double output) {}

  /** Stop all intake motors. */
  public default void stop() {}

  /** Set current limits for all intake motors. */
  public default void setCurrentLimits(double currentLimitAmps) {}

  /** Set both supply and stator current limits for all intake motors. */
  public default void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {}
}
