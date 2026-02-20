package org.frc3512.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

  @AutoLog
  public static class ConveyorIOInputs {
    public double hopperVelocity = 0.0;
    public double hopperVolts = 0.0;
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void setHopper(double speed) {}
}
