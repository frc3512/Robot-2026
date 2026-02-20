package org.frc3512.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIO_REAL implements HoodIO {

  private final TalonFX motor;

  // 0 Degres
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private double wantedDegrees = 0.0;

  public HoodIO_REAL() {
    motor = new TalonFX(HoodConstants.motorID);

    // Configure motor for position control
    motor.getConfigurator().apply(HoodConstants.config);

    motor.setPosition(0.00000);
  }

  @Override
  public void setPosition(double wantedDegrees) {
    this.wantedDegrees = wantedDegrees;
    positionRequest.Position = wantedDegrees * 360.0; // Degrees to sensor units
    motor.setControl(positionRequest);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = motor.getPosition().getValueAsDouble() * 360.0; // Degrees
    inputs.appliedVolts = motor.getStatorCurrent().getValueAsDouble();
    inputs.positionSetpoint = wantedDegrees * 360.0;
  }
}
