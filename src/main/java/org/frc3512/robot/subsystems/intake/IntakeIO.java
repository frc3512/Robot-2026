package org.frc3512.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double rollerVelocity = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerTargetSpeed = 0.0;

    public double extensionPosition = 0.0;
    public double extensionAppliedVolts = 0.0;

    public double rollerMotorTemp = 0.0;
    public double secondaryRollerMotorTemp = 0.0;
    public double extensionMotorTemp = 0.0;
    
    public boolean rollerMotorConnected = false;
    public boolean secondaryRollerMotorConnected = false;
    public boolean extensionMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setExtensionPosition(IntakeConstants.IntakeState state) {}

  public default void setExtensionPosition(double position) {}

  public default void setRollerSpeed(double speed) {}

  public default void setExtensionVelocity(double velocity) {}

  public default void rezeroExtension() {}
}
