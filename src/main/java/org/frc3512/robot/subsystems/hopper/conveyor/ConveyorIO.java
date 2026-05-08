package org.frc3512.robot.subsystems.hopper.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public boolean connected = false;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /** Run conveyor motor at the specified velocity (mechanism RPM). */
  public default void setVelocity(double mechanismRPM) {}

  /** Run conveyor motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Stop conveyor motor. */
  public default void stop() {}

  /** Set current limit for conveyor motor. */
  public default void setCurrentLimit(double currentLimitAmps) {}

  /** Set both supply and stator current limits for conveyor motor. */
  public default void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {}
}
