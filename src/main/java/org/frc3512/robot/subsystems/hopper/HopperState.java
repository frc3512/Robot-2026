package org.frc3512.robot.subsystems.hopper;

/**
 * States for the hopper subsystem state machine.
 * Controls intake and conveyor subsystems.
 */
public enum HopperState {
  /** Idle state - intake extended, slow roller speed */
  IDLE,
  
  /** Intaking state - intake at full power, extended */
  INTAKING,
  
  /** Compressing state - intake retracted to compress game pieces for shooting */
  COMPRESSING,
  
  /** Ferrying state - intake extended, robot pointed at alliance wall */
  FERRYING,
  
  /** Dumping state - everything running backwards */
  DUMPING,
  
  /** Home state - everything stopped, intake retracted */
  HOME,
  
  /** Auto state - maximum performance settings */
  AUTO
}
