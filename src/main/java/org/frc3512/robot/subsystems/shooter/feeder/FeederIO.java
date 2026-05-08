package org.frc3512.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    // Feed motor (feeds into shooter tower)
    public boolean feedConnected = false;
    public double feedVelocityRPS = 0.0;
    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;
    public double feedTempCelsius = 0.0;

    // Booster motor (below drum)
    public boolean boosterConnected = false;
    public double boosterVelocityRPS = 0.0;
    public double boosterAppliedVolts = 0.0;
    public double boosterCurrentAmps = 0.0;
    public double boosterTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run feed motor at the specified velocity (mechanism RPM). */
  public default void setFeedVelocity(double mechanismRPM) {}

  /** Run feed motor at the specified open loop value. */
  public default void setFeedOpenLoop(double output) {}

  /** Run booster motor at the specified velocity (mechanism RPM). */
  public default void setBoosterVelocity(double mechanismRPM) {}

  /** Run booster motor at the specified open loop value. */
  public default void setBoosterOpenLoop(double output) {}

  /** Stop all feeder motors. */
  public default void stop() {}

  /** Set current limit for feeder motors. */
  public default void setCurrentLimits(double currentLimitAmps) {}

  /** Set both supply and stator current limits for feeder motors. */
  public default void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {}
}
