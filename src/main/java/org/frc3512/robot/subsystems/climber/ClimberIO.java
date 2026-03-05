package org.frc3512.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double climberVelocity = 0.0;
    public double climberVolts = 0.0;

    public boolean canRaiseClimber = false;
    public boolean canLowerClimber = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setExtensionPosition(double position) {}

  public default void setClimber(double speed) {}

  public default void lowerClimber() {}

  public default void raiseClimber() {}
}
