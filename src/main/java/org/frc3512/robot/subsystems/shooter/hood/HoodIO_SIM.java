package org.frc3512.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class HoodIO_SIM implements HoodIO {

  private double currentPosition = 0.0; // Degrees
  private double wantedDegrees = 0.0;
  private double appliedVolts = 0.0;

  // PID controller for simulation
  private final PIDController pidController =
      new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

  public HoodIO_SIM() {
    // No hardware initialization needed for sim
  }

  @Override
  public void setPosition(double wantedDegrees) {
    this.wantedDegrees = wantedDegrees;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Calculate voltage needed to move towards target using PID
    appliedVolts = pidController.calculate(currentPosition, wantedDegrees);

    // Clamp voltage to reasonable simulation range (-12 to 12 volts)
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    // Simulate hood movement based on applied voltage
    // Assume max speed of ~100 degrees per second at full voltage
    double maxSpeed = 100.0; // degrees per second
    double deltaPosition = (appliedVolts / 12.0) * maxSpeed * 0.02; // Assume 20ms update cycle

    currentPosition += deltaPosition;

    // Clamp position to valid range (0 to some max degrees, typically 0-60 degrees for a hood)
    currentPosition = MathUtil.clamp(currentPosition, 0.0, 60.0);

    // Update inputs
    inputs.position = currentPosition;
    inputs.appliedVolts = appliedVolts;
    inputs.positionSetpoint = wantedDegrees;
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(currentPosition - wantedDegrees) <= HoodConstants.positionTolerance;
  }
}
