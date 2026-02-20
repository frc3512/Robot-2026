package org.frc3512.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double leftVelocity = 0.0;
    public double middleVelocity = 0.0;
    public double rightVelocity = 0.0;
    public double leftAppliedVolts = 0.0;
    public double middleAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;

    public double rpmSetpoint = 0.0;
    public boolean isVelocityWithinTolerance;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setRPM(double rpm) {}

  public default void setPercentOutput(double percent) {}

  public default void stop() {}

  public default boolean isVelocityWithinTolerance() {
    return false;
  }

  public default void setOutput(double output) {}
}
