package org.frc3512.robot.subsystems.hopper;

import org.frc3512.robot.subsystems.hopper.intake.Intake;
import org.frc3512.robot.subsystems.hopper.conveyor.Conveyor;
import org.frc3512.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * State machine for controlling hopper subsystems (intake, conveyor).
 * Handles intake and conveyor states with current limit management.
 */
public class HopperStateMachine extends SubsystemBase {
  // Subsystems
  private final Intake intake;
  private final Conveyor conveyor;
  
  // State tracking
  public HopperState currentState = HopperState.IDLE;
  public HopperState wantedState = HopperState.IDLE;
  public HopperState prevState = HopperState.IDLE;
  
  // Compression agitation tracking
  private final Timer compressionTimer = new Timer();
  private boolean compressionStarted = false;
  private boolean lastIntakeExtended = false;
  
  // Singleton pattern
  private static HopperStateMachine currentInstance;

  public HopperStateMachine(Intake intake, Conveyor conveyor) {
    this.intake = intake;
    this.conveyor = conveyor;
    
    // Set singleton instance
    currentInstance = this;
  }
  
  public static HopperStateMachine getInstance() {
    return currentInstance;
  }
  
  public static void setInstance(HopperStateMachine instance) {
    currentInstance = instance;
  }

  @Override
  public void periodic() {
    prevState = currentState;
    Logger.recordOutput("HopperStateMachine/PrevState", prevState.toString());
    Logger.recordOutput("HopperStateMachine/CurrentState", currentState.toString());
    Logger.recordOutput("HopperStateMachine/WantedState", wantedState.toString());
    
    // Reset compression tracking when exiting COMPRESSING state
    if (prevState == HopperState.COMPRESSING && currentState != HopperState.COMPRESSING) {
      compressionStarted = false;
      lastIntakeExtended = false;
    }
    
    handleStateTransitions();
    applyStates();
  }

  /**
   * Handles state transitions with conditions.
   * This method checks if we can transition to the wanted state.
   */
  public void handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        if (canEnterIdle()) {
          currentState = HopperState.IDLE;
        }
        break;
        
      case INTAKING:
        if (canEnterIntaking()) {
          currentState = HopperState.INTAKING;
        }
        break;
        
      case COMPRESSING:
        if (canEnterCompressing()) {
          currentState = HopperState.COMPRESSING;
        }
        break;
        
      case FERRYING:
        if (canEnterFerrying()) {
          currentState = HopperState.FERRYING;
        }
        break;
        
      case DUMPING:
        if (canEnterDumping()) {
          currentState = HopperState.DUMPING;
        }
        break;
        
      case HOME:
        if (canEnterHome()) {
          currentState = HopperState.HOME;
        }
        break;
        
      case AUTO:
        if (canEnterAuto()) {
          currentState = HopperState.AUTO;
        }
        break;
        
      default:
        throw new RuntimeException("Invalid hopper state: " + wantedState.toString());
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
        
      case INTAKING:
        stateINTAKING();
        break;
        
      case COMPRESSING:
        stateCOMPRESSING();
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
        throw new RuntimeException("Invalid hopper state: " + currentState.toString());
    }
  }
  
  /**
   * Sets the wanted state. State transition will be handled in handleStateTransitions().
   */
  public void setWantedState(HopperState state) {
    wantedState = state;
    Logger.recordOutput("HopperStateMachine/WantedStateSet", state.toString());
  }
  
  /**
   * Legacy method for backward compatibility. Use setWantedState() instead.
   */
  public void setState(HopperState state) {
    setWantedState(state);
  }

  
  public HopperState getState() {
    return currentState;
  }
  
  public HopperState getWantedState() {
    return wantedState;
  }

  // State execution methods
  
  private void stateIDLE() {
    // Set subsystems to idle configurations
    intake.extend();
    intake.setRollerVelocity(100.0); // Very slow roller speed
    conveyor.setVelocity(0.0); // Stop conveyor
  }
  
  private void stateINTAKING() {
    // Intake at full power, extended
    intake.extend();
    intake.setRollerVelocity(3000.0); // Near full power
    conveyor.setVelocity(200.0); // Slowly run conveyor
  }
  
  private void stateCOMPRESSING() {
    // Start compression sequence when entering state
    if (prevState != HopperState.COMPRESSING) {
      intake.retract();
      intake.setRollerVelocity(100.0); // Slow rollers to hold pieces
      // Don't control conveyor - ShootingController handles it during shooting
      compressionStarted = true;
      compressionTimer.restart();
    }
    
    // Phase 2: Intake agitation 0.25s after compression starts
    if (compressionStarted && compressionTimer.get() >= ShootingConstants.INTAKE_AGITATION_START) {
      double agitateElapsed = compressionTimer.get() - ShootingConstants.INTAKE_AGITATION_START;
      boolean targetIntakeExtended =
          (agitateElapsed % ShootingConstants.INTAKE_AGITATION_PERIOD < ShootingConstants.INTAKE_AGITATION_PERIOD / 2.0);
      if (targetIntakeExtended != lastIntakeExtended) {
        if (targetIntakeExtended) {
          intake.extend();
        } else {
          intake.setExtensionPosition(6.0); // Mid position for agitation
        }
        lastIntakeExtended = targetIntakeExtended;
      }
    }
  }
  
  private void stateFERRYING() {
    // Intake extended and intaking
    intake.extend();
    intake.setRollerVelocity(3000.0);
    
    // Run conveyor
    conveyor.setVelocity(500.0);
  }
  
  private void stateDUMPING() {
    // Run everything backwards slowly
    intake.extend();
    intake.setRollerVelocity(-500.0); // Backwards
    
    conveyor.setVelocity(-200.0); // Backwards
  }
  
  private void stateHOME() {
    // Stop everything and move to home positions
    intake.retract();
    intake.stop();
    conveyor.stop();
  }
  
  private void stateAUTO() {
    // Maximum performance settings
    intake.extend();
    intake.setRollerVelocity(2000.0);
    
    conveyor.setVelocity(500.0);
  }
  
  // Condition checking methods
  
  private boolean canEnterIdle() {
    return true; // Always can enter idle
  }
  
  private boolean canEnterIntaking() {
    // Add conditions like: intake is not damaged, sensors are working, etc.
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  private boolean canEnterCompressing() {
    // Can compress if intake is functional
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  private boolean canEnterFerrying() {
    // Add conditions like: both intake and conveyor are ready
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  private boolean canEnterDumping() {
    // Add conditions like: path is clear, no obstructions, etc.
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  private boolean canEnterHome() {
    // Add conditions like: no game pieces in system, etc.
    return true; // Always can home
  }
  
  private boolean canEnterAuto() {
    // Add conditions like: all systems are ready for auto
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  // Legacy getters for backward compatibility

  @AutoLogOutput
  public HopperState getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
  public boolean isIntakeExtended() {
    return intake.isExtended();
  }

  @AutoLogOutput
  public boolean isIntakeRetracted() {
    return intake.isRetracted();
  }

  @AutoLogOutput
  public double getIntakeRollerVelocityRPM() {
    return intake.getAverageRollerVelocityRPM();
  }

  @AutoLogOutput
  public double getConveyorVelocityRPM() {
    return conveyor.getVelocityRPM();
  }
}
