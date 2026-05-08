package org.frc3512.robot.subsystems.shooter;

/**
 * States for the shooter subsystem state machine.
 * Controls hood, feeder, and drum subsystems.
 */
public enum ShooterState {
  /** Idle state - minimum shooting speed, hood at minimum angle */
  IDLE,
  
  /** Spinning state - drum spinning up, hood at shooting angle */
  SPINNING,
  
  /** Aiming state - using ShootingController to aim at hub */
  AIMING,
  
  /** Shooting state - actively shooting balls */
  SHOOTING,
  
  /** Ferrying state - static shooter settings for ferrying */
  FERRYING,
  
  /** Dumping state - running everything backwards */
  DUMPING,
  
  /** Home state - everything stopped and retracted */
  HOME,
  
  /** Auto state - maximum performance settings */
  AUTO
}
