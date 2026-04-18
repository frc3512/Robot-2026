package org.frc3512.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private FeederStates currentState = FeederStates.STOPPED;
  private FeederStates wantedState = FeederStates.STOPPED;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void setFeeder(double speed) {
    io.setFeeder(speed);
  }

  public FeederStates getCurrentState() {
    return currentState;
  }

  public FeederStates getWantedState() {
    return wantedState;
  }

  public void setWantedState(FeederStates state) {
    wantedState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    handleStateTransitions();
    applyStates();

  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case STOPPED:
        currentState = FeederStates.STOPPED;
        break;
      case FEEDING:
        currentState = FeederStates.FEEDING;
        break;
      case OUTAKING:
        currentState = FeederStates.OUTAKING;
        break;
    }
  }

  private void applyStates() {
    switch (currentState) {
      case STOPPED:
        io.setFeeder(0);
        break;
      case FEEDING:
        io.setFeeder(0.75);
        break;
      case OUTAKING:
        io.setFeeder(-0.5);
        break;
    }
  }

}
