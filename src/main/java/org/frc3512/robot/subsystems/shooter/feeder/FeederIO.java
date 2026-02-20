package org.frc3512.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double feederAppliedOutput = 0.0;
    public double feederVelocity = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeeder(double speed) {}
}
