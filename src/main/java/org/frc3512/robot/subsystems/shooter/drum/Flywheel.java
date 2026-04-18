package org.frc3512.robot.subsystems.shooter.drum;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private FlywheelStates currentState = FlywheelStates.OFF;
  private FlywheelStates wantedState = FlywheelStates.IDLE;
  private double targetRPM = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    wantedState = FlywheelStates.ACCELERATING;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void setWantedState(FlywheelStates state) {
    wantedState = state;
  }

  public boolean isAtSetpoint() {
    return io.isVelocityWithinTolerance();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    handleStateTransitions();
    applyOutput();
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case ACCELERATING:
        if (targetRPM > 0.0 && ! io.isVelocityWithinTolerance()) {
          currentState = FlywheelStates.ACCELERATING;
        }
        break;
      case IDLE:
        if (targetRPM == 1800.0 && io.isVelocityWithinTolerance()) {
          currentState = FlywheelStates.IDLE;
        }
        break;
      case READY:
        if (io.isVelocityWithinTolerance()) {
          currentState = FlywheelStates.READY;
        }
        break;
      case OFF:
        if (targetRPM == 0.0) {
          currentState = FlywheelStates.OFF;
        }
        break;
    }
  }

  private void applyOutput() {
    switch (currentState) {
      case ACCELERATING:
        io.setRPM(targetRPM); 
        break;
      case IDLE:
        io.setRPM(1800.0);
        break;
      case READY:
        io.setRPM(targetRPM);
        break;
      case OFF:
        io.setRPM(0.0);
        break;
    }
  }
}
