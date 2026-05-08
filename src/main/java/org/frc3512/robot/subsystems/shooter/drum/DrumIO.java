package org.frc3512.robot.subsystems.shooter.drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {
  @AutoLog
  public static class DrumIOInputs {
    // Motors on one side (2 motors)
    public boolean motor1Connected = false;
    public double motor1VelocityRPS = 0.0;
    public double motor1AppliedVolts = 0.0;
    public double motor1CurrentAmps = 0.0;
    public double motor1TempCelsius = 0.0;

    public boolean motor2Connected = false;
    public double motor2VelocityRPS = 0.0;
    public double motor2AppliedVolts = 0.0;
    public double motor2CurrentAmps = 0.0;
    public double motor2TempCelsius = 0.0;

    // Motor on other side (1 motor)
    public boolean motor3Connected = false;
    public double motor3VelocityRPS = 0.0;
    public double motor3AppliedVolts = 0.0;
    public double motor3CurrentAmps = 0.0;
    public double motor3TempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrumIOInputs inputs) {}

  /** Run all drum motors at the specified velocity (mechanism RPM). */
  public default void setVelocity(double mechanismRPM) {}

  /** Run all drum motors at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Set both supply and stator current limits for the drum motor. */
  public default void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {}

  /** Stop all drum motors. */
  public default void stop() {}
}
