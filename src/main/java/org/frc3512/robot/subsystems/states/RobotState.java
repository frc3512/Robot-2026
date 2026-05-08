package org.frc3512.robot.subsystems.states;

/**
 * Enum representing the main robot states for the state machine.
 * Each state defines the behavior of all subsystems.
 */
public enum RobotState {
  /** Idle state - minimal operation, ready for action */
  IDLE,
  
  /** Intake state - actively collecting game pieces */
  INTAKING,
  
  /** Aiming state - positioning shooter for optimal shot */
  AIMING,
  
  /** Shooting state - actively shooting game pieces */
  SHOOTING,
  
  /** Ferrying state - moving pieces while maintaining shooting capability */
  FERRYING,
  
  /** Dumping state - expelling game pieces backwards */
  DUMPING,
  
  /** Home state - reset all subsystems to safe positions */
  HOME,
  
  /** Auto state - maximum performance for autonomous */
  AUTO
}
