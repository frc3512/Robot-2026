package org.frc3512.robot.commands;

import org.frc3512.robot.subsystems.statemachine.MasterStateMachine;
import org.frc3512.robot.subsystems.states.RobotState;

/**
 * State command implementations for robot state transitions.
 * These are NOT WPI commands - they are simple state transition methods
 * that can be called directly from button bindings.
 */
public class StateCommands {
  private final MasterStateMachine stateMachine;

  public StateCommands(MasterStateMachine stateMachine) {
    this.stateMachine = stateMachine;
  }

  /**
   * Transition to IDLE state.
   * Used for: Right bumper (WhileTrue), A button (OnFalse)
   */
  public void toIdle() {
    stateMachine.setWantedState(RobotState.IDLE);
  }

  /**
   * Transition to INTAKING state.
   * Used for: B button (WhileTrue), Right bumper (OnFalse)
   */
  public void toIntaking() {
    stateMachine.setWantedState(RobotState.INTAKING);
  }

  /**
   * Transition to AIMING state.
   * Used for: X button (WhileTrue)
   */
  public void toAiming() {
    stateMachine.setWantedState(RobotState.AIMING);
  }

  /**
   * Transition to SHOOTING state.
   * Used for: Y button (WhileTrue), X button (OnFalse)
   * Sets shooting button held and transitions to aiming first, then auto-shoots when ready.
   */
  public void toShooting() {
    stateMachine.wantToShoot(true);
    stateMachine.setWantedState(RobotState.AIMING); // Start with aiming
  }
  
  /**
   * Stop shooting.
   * Call this when shooting button is released.
   */
  public void stopShooting() {
    stateMachine.wantToShoot(false);
  }

  /**
   * Transition to FERRYING state.
   * Used for: Left bumper (WhileTrue)
   */
  public void toFerrying() {
    stateMachine.setWantedState(RobotState.FERRYING);
  }

  /**
   * Transition to DUMPING state.
   * Used for: B button (WhileTrue), Left bumper (OnFalse)
   */
  public void toDumping() {
    stateMachine.setWantedState(RobotState.DUMPING);
  }

  /**
   * Transition to HOME state.
   * Used for: A button (WhileTrue), Y button (OnFalse)
   */
  public void toHome() {
    stateMachine.setWantedState(RobotState.HOME);
  }

  /**
   * Transition to AUTO state.
   * Used for: Start button (WhileTrue)
   */
  public void toAuto() {
    stateMachine.setWantedState(RobotState.AUTO);
  }

  /**
   * Get current robot state.
   */
  public RobotState getCurrentState() {
    return stateMachine.getState();
  }

  /**
   * Check if currently aiming.
   */
  public boolean isAiming() {
    return stateMachine.isAiming();
  }

  /**
   * Check if ready to shoot.
   */
  public boolean isReadyToShoot() {
    return stateMachine.isReadyToShoot();
  }
}
