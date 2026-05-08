package org.frc3512.robot.subsystems.shooter;

import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.constants.MechanicalConstants;
import org.frc3512.robot.ai.EnhancedShootingController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * State machine for controlling shooter subsystems (hood, feeder, drum).
 * Handles shooting states and integrates with ShootingController for advanced aiming.
 */

@SuppressWarnings("unused")
public class ShooterStateMachine extends SubsystemBase {
  // Subsystems
  private final Drum drum;
  private final Feeder feeder;
  private final Hood hood;
  private final Drive drive;
  
  // Shooting controller
  private final ShootingController shootingController;
  
  // State tracking
  public ShooterState currentState = ShooterState.IDLE;
  public ShooterState wantedState = ShooterState.IDLE;
  public ShooterState prevState = ShooterState.IDLE;
  
  // Singleton pattern
  private static ShooterStateMachine currentInstance;
  
  // Enhanced shooting controller
  private EnhancedShootingController enhancedShootingController;

  public ShooterStateMachine(
      Drum drum,
      Feeder feeder,
      Hood hood,
      Drive drive) {
    this.drum = drum;
    this.feeder = feeder;
    this.hood = hood;
    this.drive = drive;
    
    // Initialize shooting controller without enhanced controller (will be set later)
    this.shootingController = new ShootingController(drive, drum, hood, feeder, null);
    
    // Set singleton instance
    currentInstance = this;
  }
  
  /**
   * Sets the enhanced shooting controller.
   */
  public void setEnhancedShootingController(EnhancedShootingController enhancedController) {
    this.enhancedShootingController = enhancedController;
  }
  
  public static ShooterStateMachine getInstance() {
    return currentInstance;
  }
  
  public static void setInstance(ShooterStateMachine instance) {
    currentInstance = instance;
  }

  /**
   * Helper method to stop shooting controller if active.
   */
  private void stopShootingController() {
    if (shootingController.isActive()) {
      shootingController.stop();
    }
  }

  @Override
  public void periodic() {
    prevState = currentState;
    Logger.recordOutput("ShooterStateMachine/PrevState", prevState.toString());
    Logger.recordOutput("ShooterStateMachine/CurrentState", currentState.toString());
    Logger.recordOutput("ShooterStateMachine/WantedState", wantedState.toString());
    
    handleStateTransitions();
    applyStates();
    
    // Log shooting controller status
    Logger.recordOutput("ShooterStateMachine/Aiming", shootingController.isAimed());
    Logger.recordOutput("ShooterStateMachine/ReadyToShoot", shootingController.isReadyToShoot());
  }

  /**
   * Handles state transitions with conditions.
   * This method checks if we can transition to the wanted state.
   */
  public void handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        if (canEnterIdle()) {
          currentState = ShooterState.IDLE;
        }
        break;
        
      case SPINNING:
        if (canEnterSpinning()) {
          currentState = ShooterState.SPINNING;
        }
        break;
        
      case AIMING:
        if (canEnterAiming()) {
          currentState = ShooterState.AIMING;
        }
        break;
        
      case SHOOTING:
        if (canEnterShooting()) {
          currentState = ShooterState.SHOOTING;
        }
        break;
        
      case FERRYING:
        if (canEnterFerrying()) {
          currentState = ShooterState.FERRYING;
        }
        break;
        
      case DUMPING:
        if (canEnterDumping()) {
          currentState = ShooterState.DUMPING;
        }
        break;
        
      case HOME:
        if (canEnterHome()) {
          currentState = ShooterState.HOME;
        }
        break;
        
      case AUTO:
        if (canEnterAuto()) {
          currentState = ShooterState.AUTO;
        }
        break;
        
      default:
        throw new RuntimeException("Invalid shooter state: " + wantedState.toString());
    }
  }
  
  /**
   * Applies the current state by executing appropriate state methods.
   */
  public void applyStates() {
    switch (currentState) {
      case IDLE:
        stateIDLE();
        break;
        
      case SPINNING:
        stateSPINNING();
        break;
        
      case AIMING:
        stateAIMING();
        break;
        
      case SHOOTING:
        stateSHOOTING();
        break;
        
      case FERRYING:
        stateFERRYING();
        break;
        
      case DUMPING:
        stateDUMPING();
        break;
        
      case HOME:
        stateHOME();
        break;
        
      case AUTO:
        stateAUTO();
        break;
        
      default:
        throw new RuntimeException("Invalid shooter state: " + currentState.toString());
    }
  }
  
  /**
   * Sets the wanted state. State transition will be handled in handleStateTransitions().
   */
  public void setWantedState(ShooterState state) {
    wantedState = state;
    Logger.recordOutput("ShooterStateMachine/WantedStateSet", state.toString());
  }
  
  /**
   * Legacy method for backward compatibility. Use setWantedState() instead.
   */
  public void setState(ShooterState state) {
    setWantedState(state);
  }

  
  public ShooterState getState() {
    return currentState;
  }
  
  public ShooterState getWantedState() {
    return wantedState;
  }

  // State execution methods
  
  private void stateIDLE() {
    stopShootingController();
    
    // Set subsystems to idle configurations
    drum.setVelocity(500.0); // Minimum shooting speed
    hood.setAngle(MechanicalConstants.Hood.MIN_ANGLE_DEGREES);
    feeder.setFeedVelocity(0.0);
    feeder.setBoosterVelocity(0.0);
  }
  
  private void stateSPINNING() {
    stopShootingController();
    
    // Spin up drum to shooting speed
    drum.setVelocity(2000.0); // Full shooting speed
    hood.setAngle(30.0); // Shooting angle
    feeder.setBoosterVelocity(500.0); // Speed up pusher motor
  }
  
  private void stateAIMING() {
    // Start shooting controller - it will handle all aiming logic
    if (!shootingController.isActive()) {
      shootingController.start();
    }
    shootingController.update();
  }
  
  private void stateSHOOTING() {
    // Shooting controller should already be active from aiming state
    // It will handle the shooting sequence automatically
    if (!shootingController.isActive()) {
      shootingController.start();
    }
    shootingController.update();
  }
  
  private void stateFERRYING() {
    stopShootingController();
    
    // Static shooter settings
    drum.setVelocity(1500.0);
    hood.setAngle(30.0);
    feeder.setFeedVelocity(1000.0);
    feeder.setBoosterVelocity(500.0);
  }
  
  private void stateDUMPING() {
    stopShootingController();
    
    // Run everything backwards slowly
    drum.setVelocity(300.0);
    hood.setAngle(15.0);
    feeder.setFeedVelocity(-500.0);
    feeder.setBoosterVelocity(-500.0);
  }
  
  private void stateHOME() {
    stopShootingController();
    
    // Stop everything and move to home positions
    drum.stop();
    hood.setAngle(MechanicalConstants.Hood.MIN_ANGLE_DEGREES);
    feeder.stop();
  }
  
  private void stateAUTO() {
    stopShootingController();
    
    // Maximum performance settings
    drum.setVelocity(3000.0);
    hood.setAngle(30.0);
    feeder.setFeedVelocity(1000.0);
    feeder.setBoosterVelocity(500.0);
  }
  
  // Condition checking methods
  
  private boolean canEnterIdle() {
    return true; // Always can enter idle
  }
  
  private boolean canEnterSpinning() {
    // Add conditions like: drum is healthy, no obstructions, etc.
    return drum.areMotorsConnected();
  }
  
  private boolean canEnterAiming() {
    // Add conditions like: vision is working, shooter is ready, etc.
    return drum.areMotorsConnected() && hood.isConnected();
  }
  
  private boolean canEnterShooting() {
    // Add conditions like: shooter is aimed, game piece is ready, etc.
    return shootingController.isReadyToShoot();
  }
  
  private boolean canEnterFerrying() {
    // Add conditions like: shooter is ready, feeder is ready, etc.
    return drum.areMotorsConnected() && feeder.areMotorsConnected();
  }
  
  private boolean canEnterDumping() {
    // Add conditions like: path is clear, no obstructions, etc.
    return drum.areMotorsConnected() && feeder.areMotorsConnected();
  }
  
  private boolean canEnterHome() {
    // Add conditions like: no game pieces in system, etc.
    return true; // Always can home
  }
  
  private boolean canEnterAuto() {
    // Add conditions like: all systems are ready for auto
    return drum.areMotorsConnected() && 
           feeder.areMotorsConnected() && 
           hood.isConnected();
  }
  
  // Legacy getters for backward compatibility

  @AutoLogOutput
  public boolean isAiming() {
    return shootingController.isAimed();
  }

  @AutoLogOutput
  public boolean isReadyToShoot() {
    return shootingController.isReadyToShoot();
  }

  @AutoLogOutput
  public ShooterState getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
  public double getTargetRPM() {
    return shootingController.getTargetRPM();
  }

  @AutoLogOutput
  public double getTargetAngle() {
    return shootingController.getTargetAngle();
  }

  @AutoLogOutput
  public double getCompensatedDistance() {
    return shootingController.getCompensatedDistance();
  }
}
