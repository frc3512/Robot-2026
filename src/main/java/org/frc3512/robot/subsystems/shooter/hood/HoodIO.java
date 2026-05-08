package org.frc3512.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Run hood motor to the specified angle (degrees). */
  public default void setAngle(double degrees) {}

  /** Run hood motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Stop hood motor. */
  public default void stop() {}
}
