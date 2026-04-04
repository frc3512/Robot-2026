package org.frc3512.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIO_REAL implements HoodIO {

  private final TalonFX motor;

  // 0 Degres
  private PositionVoltage positionRequest = new PositionVoltage(0.0);
  private double wantedPos;

  public HoodIO_REAL() {
    // Initialize motor and apply configuration
    motor = new TalonFX(HoodConstants.motorID);
    motor.getConfigurator().apply(HoodConstants.config);

    // Set initial position to 10 degrees as minimum shooting angle
    motor.setPosition(10.000000000);

    // Optimize bus utilization for better performance
    motor.optimizeBusUtilization();
  }

  @Override
  public void setPosition(double wantedDegrees) {
    wantedPos = wantedDegrees;
  }

  @Override
  public boolean isAtTarget() {
    double currentPosition = motor.getPosition().getValueAsDouble() * 360.0;
    return Math.abs((currentPosition - wantedPos / 360.0)) < HoodConstants.positionTolerance;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Update motor control
    motor.setControl(positionRequest.withPosition(wantedPos / 360.0));
    // Update motor states and sensor readings
    inputs.position = motor.getPosition().getValueAsDouble() * 360.0;
    inputs.appliedVolts = motor.getStatorCurrent().getValueAsDouble();
    inputs.positionSetpoint = wantedPos;

    // Check if the hood is at the target position and update the input accordingly
    inputs.isAtTarget = isAtTarget();
  }
}
