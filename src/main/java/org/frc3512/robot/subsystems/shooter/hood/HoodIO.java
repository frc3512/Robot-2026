package org.frc3512.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double positionSetpoint = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default boolean isAtTarget() {
    return false;
  }
}
