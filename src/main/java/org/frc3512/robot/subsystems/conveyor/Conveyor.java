package org.frc3512.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

  private ConveyorIO io;
  private ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private ConveyorStates currentState = ConveyorStates.STOPPED;
  private ConveyorStates wantedState = ConveyorStates.STOPPED;

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);

    handleStateTransitions();
    applyStates();
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case FEEDING:
        currentState = ConveyorStates.FEEDING;
        break;
      case OUTTAKING:
        currentState = ConveyorStates.OUTTAKING;
        break;
      case STOPPED:
        currentState = ConveyorStates.STOPPED;
        break;
    }
  }

  private void applyStates() {
    switch (currentState) {
      case FEEDING:
        io.setHopper(0.75);
        break;
      case OUTTAKING:
        io.setHopper(-0.5);
        break;
      case STOPPED:
        io.setHopper(0.0);
        break;
    }
  }

  public void setWantedState(ConveyorStates state) {
    wantedState = state;
  }

  @AutoLogOutput(key = "Conveyor/State")
  public ConveyorStates getCurrentState() {
    return currentState;
  }

  @AutoLogOutput(key = "Conveyor/WantedState")
  public ConveyorStates getWantedState() {
    return wantedState;
  }
}
