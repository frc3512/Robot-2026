package org.frc3512.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeStates currentState = IntakeStates.IDLE;
  private IntakeStates wantedState = IntakeStates.IDLE;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setWantedState(IntakeStates state) {
    this.wantedState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    handleStateTransitions();
    applyStates();
  }

  private void handleStateTransitions() {
    switch(wantedState) {
      case HOME:
        currentState = IntakeStates.HOME;
        break;
      case IDLE:
        currentState = IntakeStates.IDLE;
        break;
      case INTAKING:
        currentState = IntakeStates.INTAKING;
        break;
      case OUTAKING:
        currentState = IntakeStates.OUTAKING;
        break;
      case COMPRESSING:
        currentState = IntakeStates.COMPRESSING;
        break;
    }
  }

  private void applyStates() {
    switch(currentState) {
      case HOME:
        io.setExtensionPosition(IntakeConstants.IntakePosition.STOWED);
        io.setRollerSpeed(0);
        break;
      case IDLE:
        io.setExtensionPosition(IntakeConstants.IntakePosition.EXTEND);
        io.setRollerSpeed(0.01);
        break;
      case INTAKING:
        io.setExtensionPosition(IntakeConstants.IntakePosition.EXTEND);
        io.setRollerSpeed(0.85);
        break;
      case OUTAKING:
        io.setExtensionPosition(IntakeConstants.IntakePosition.EXTEND);
        io.setRollerSpeed(-0.85);
        break;
      case COMPRESSING:
        io.compressIntake();
        break;
    }
  }
}
