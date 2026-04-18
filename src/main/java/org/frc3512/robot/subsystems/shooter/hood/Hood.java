package org.frc3512.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodStates currentState = HoodStates.IDLE;
  private HoodStates wantedState = HoodStates.IDLE;
  private double targetPosition = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setPosition(double position) {
    io.setPosition(position);
    wantedState = HoodStates.AIMING;
  }

  public HoodStates getCurrentState() {
    return currentState;
  }

  public HoodStates getWantedState() {
    return wantedState;
  }

  public void setWantedState(HoodStates state) {
    wantedState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    handleStateTransitions();
    applyStates();
  }

  private void handleStateTransitions() {
    switch(wantedState) {
      case HOME:
        currentState = HoodStates.HOME;
        break;
      case IDLE:
        currentState = HoodStates.IDLE;
        break;
      case AIMING:
        currentState = HoodStates.AIMING;
        break;
    }
  }

  private void applyStates() {
    switch(currentState) {
      case HOME:
        io.setPosition(10.0); // Home position
        break;
      case IDLE:
        // Hold current position, no movement
        break;
      case AIMING:
        io.setPosition(targetPosition);
        break;
    }
  }
}
