package org.frc3512.robot.subsystems.statemachine;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.shooter.ShooterState;
import org.frc3512.robot.subsystems.shooter.ShooterStateMachine;
import org.frc3512.robot.subsystems.hopper.HopperState;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.states.RobotState;
import org.frc3512.robot.subsystems.hopper.intake.Intake;
import org.frc3512.robot.subsystems.hopper.HopperStateMachine;
import org.frc3512.robot.subsystems.hopper.conveyor.Conveyor;
import org.frc3512.robot.subsystems.leds.LED;
import org.frc3512.robot.subsystems.leds.LEDPatterns;
import org.frc3512.robot.subsystems.vision.Vision;
import org.frc3512.robot.subsystems.vision.VisionConstants;
import org.frc3512.robot.RobotContainer;
import org.frc3512.robot.constants.CurrentLimits;
import org.frc3512.robot.ai.EnhancedShootingController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Master state machine that coordinates shooter and hopper state machines plus drivetrain.
 * Provides unified interface for robot-wide state management and current limit control.
 */
public class MasterStateMachine extends SubsystemBase {
  // Subsystems needed for current limit updates
  private final Drive drive;
  private final Drum drum;
  private final Feeder feeder;
  private final Intake intake;
  private final Conveyor conveyor;
  private final LED leds;
  private final Hood hood;
  private final Vision vision;
  private final RobotContainer robotContainer;
  
  // State machines
  private final ShooterStateMachine shooterStateMachine;
  private final HopperStateMachine hopperStateMachine;
  
  // State tracking
  public RobotState currentState = RobotState.IDLE;
  public RobotState wantedState = RobotState.IDLE;
  public RobotState prevState = RobotState.IDLE;
  
  // Shooting button tracking
  private boolean wantToShoot = false;
  private boolean autoShootingEnabled = true; // Enable auto-transition from aiming to shooting
  
  // Singleton pattern
  private static MasterStateMachine currentInstance;

  public MasterStateMachine(
      Drive drive,
      Drum drum,
      Feeder feeder,
      Hood hood,
      Intake intake,
      Conveyor conveyor,
      LED leds,
      Vision vision,
      RobotContainer robotContainer) {
    // Store subsystem references for current limit updates
    this.drive = drive;
    this.drum = drum;
    this.feeder = feeder;
    this.intake = intake;
    this.conveyor = conveyor;
    this.leds = leds;
    this.hood = hood;
    this.vision = vision;
    this.robotContainer = robotContainer;
    
    // Create state machines
    this.shooterStateMachine = new ShooterStateMachine(drum, feeder, hood, drive);
    this.hopperStateMachine = new HopperStateMachine(intake, conveyor);
    
    // Set singleton instance
    currentInstance = this;
  }
  
  /**
   * Sets the enhanced shooting controller in the shooter state machine.
   */
  public void setEnhancedShootingController(EnhancedShootingController enhancedController) {
    if (shooterStateMachine != null) {
      shooterStateMachine.setEnhancedShootingController(enhancedController);
    }
  }
  
  public static MasterStateMachine getInstance() {
    return currentInstance;
  }
  
  public static void setInstance(MasterStateMachine instance) {
    currentInstance = instance;
  }

  @Override
  public void periodic() {
    prevState = currentState;
    Logger.recordOutput("MasterStateMachine/PrevState", prevState.toString());
    Logger.recordOutput("MasterStateMachine/CurrentState", currentState.toString());
    Logger.recordOutput("MasterStateMachine/WantedState", wantedState.toString());
    
    handleStateTransitions();
    applyStates();
    
    // Update current limits based on robot-wide state
    updateCurrentLimits();
    
    // Log subsystem states
    Logger.recordOutput("MasterStateMachine/ShooterState", shooterStateMachine.getCurrentState().toString());
    Logger.recordOutput("MasterStateMachine/HopperState", hopperStateMachine.getCurrentState().toString());
  }

  /**
   * Handles state transitions with conditions.
   * This method checks if we can transition to the wanted state.
   */
  public void handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        if (canEnterIdle()) {
          currentState = RobotState.IDLE;
        }
        break;
        
      case INTAKING:
        if (canEnterIntaking()) {
          currentState = RobotState.INTAKING;
        }
        break;
        
      case AIMING:
        if (canEnterAiming()) {
          currentState = RobotState.AIMING;
          
          // Auto-transition to shooting if button held and criteria met
          if (autoShootingEnabled && wantToShoot && canEnterShooting()) {
            currentState = RobotState.SHOOTING;
            Logger.recordOutput("MasterStateMachine/AutoTransition", "AIMING -> SHOOTING (auto)");
          }
        }
        break;
        
      case SHOOTING:
        if (canEnterShooting()) {
          currentState = RobotState.SHOOTING;
        }
        break;
        
      case FERRYING:
        if (canEnterFerrying()) {
          currentState = RobotState.FERRYING;
        }
        break;
        
      case DUMPING:
        if (canEnterDumping()) {
          currentState = RobotState.DUMPING;
        }
        break;
        
      case HOME:
        if (canEnterHome()) {
          currentState = RobotState.HOME;
        }
        break;
        
      case AUTO:
        if (canEnterAuto()) {
          currentState = RobotState.AUTO;
        }
        break;
        
      default:
        throw new RuntimeException("Invalid robot state: " + wantedState.toString());
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
        throw new RuntimeException("Invalid robot state: " + currentState.toString());
    }
  }
  
  /**
   * Sets the wanted state. State transition will be handled in handleStateTransitions().
   */
  public void setWantedState(RobotState state) {
    wantedState = state;
    Logger.recordOutput("MasterStateMachine/WantedStateSet", state.toString());
  }
  
  /**
   * Legacy method for backward compatibility. Use setWantedState() instead.
   */
  public void setState(RobotState state) {
    setWantedState(state);
  }

  /**
   * Updates current limits for all subsystems based on current robot state.
   */
  private void updateCurrentLimits() {
    // Update shooter subsystems
    drum.setRobotState(currentState);
    feeder.setRobotState(currentState);
    
    // Update hopper subsystems  
    intake.setRobotState(currentState);
    conveyor.setRobotState(currentState);
    
    // Update drivetrain current limits
    double driveSupplyLimit = CurrentLimits.Drive.getSupplyForState(currentState);
    double driveStatorLimit = CurrentLimits.Drive.getStatorForState(currentState);
    // Note: Drive subsystem handles both supply and stator current limits internally
    Logger.recordOutput("MasterStateMachine/DriveSupplyLimit", driveSupplyLimit);
    Logger.recordOutput("MasterStateMachine/DriveStatorLimit", driveStatorLimit);
  }

  // State execution methods
  
  private void stateIDLE() {
    shooterStateMachine.setState(ShooterState.IDLE);
    hopperStateMachine.setState(HopperState.IDLE);
    updateLEDs();
  }
  
  private void stateINTAKING() {
    shooterStateMachine.setState(ShooterState.SPINNING);
    hopperStateMachine.setState(HopperState.INTAKING);
    updateLEDs();
  }
  
  private void stateAIMING() {
    shooterStateMachine.setState(ShooterState.AIMING);
    hopperStateMachine.setState(HopperState.IDLE);
    updateLEDs();
  }
  
  private void stateSHOOTING() {
    shooterStateMachine.setState(ShooterState.SHOOTING);
    hopperStateMachine.setState(HopperState.COMPRESSING);
    updateLEDs();
  }
  
  private void stateFERRYING() {
    shooterStateMachine.setState(ShooterState.FERRYING);
    hopperStateMachine.setState(HopperState.FERRYING);
    updateLEDs();
  }
  
  private void stateDUMPING() {
    shooterStateMachine.setState(ShooterState.DUMPING);
    hopperStateMachine.setState(HopperState.DUMPING);
    updateLEDs();
  }
  
  private void stateHOME() {
    shooterStateMachine.setState(ShooterState.HOME);
    hopperStateMachine.setState(HopperState.HOME);
    updateLEDs();
  }
  
  private void stateAUTO() {
    shooterStateMachine.setState(ShooterState.AUTO);
    hopperStateMachine.setState(HopperState.AUTO);
    updateLEDs();
  }
  
  // Condition checking methods
  
  private boolean canEnterIdle() {
    return true; // Always can enter idle
  }
  
  private boolean canEnterIntaking() {
    // Add conditions like: intake is not damaged, sensors are working, etc.
    return intake.areRollersConnected() && intake.isExtensionConnected();
  }
  
  private boolean canEnterAiming() {
    // Add conditions like: shooter is ready, vision is working, etc.
    // Check if we are inside alliance zone using vision
    boolean basicHardwareReady = drum.areMotorsConnected() && hood.isConnected();
    boolean inAllianceZone = isInAllianceZone();
    
    return basicHardwareReady && inAllianceZone;
  }
  
  private boolean canEnterShooting() {
    // Add conditions like: shooter is aimed, game piece is ready, etc.
    // Check if flywheel is at setpoint, hood at right angle, and robot pointed at hub
    boolean shooterReady = shooterStateMachine.isReadyToShoot();
    boolean flywheelAtSetpoint = isFlywheelAtSetpoint();
    boolean hoodAtCorrectAngle = isHoodAtCorrectAngle();
    boolean aimedAtHub = isAimedAtHub();
    
    return shooterReady && flywheelAtSetpoint && hoodAtCorrectAngle && aimedAtHub;
  }
  
  private boolean canEnterFerrying() {
    // Add conditions like: both intake and shooter are ready
    return intake.areRollersConnected() && intake.isExtensionConnected() && drum.areMotorsConnected() && isFlywheelAtSetpoint();
  }
  
  private boolean canEnterDumping() {
    // Add conditions like: path is clear, no obstructions, etc.
    return intake.areRollersConnected() && intake.isExtensionConnected() && feeder.areMotorsConnected();
  }
  
  private boolean canEnterHome() {
    // Add conditions like: no game pieces in system, etc.
    return true; // Always can home
  }
  
  private boolean canEnterAuto() {
    // Add conditions like: all systems are ready for auto
    // Check all hardware is connected AND auto mode is selected
    boolean hardwareReady = intake.areRollersConnected() && 
                          intake.isExtensionConnected() &&
                          drum.areMotorsConnected() && 
                          feeder.areMotorsConnected() && 
                          hood.isConnected();
    boolean autoModeSelected = isAutoModeSelected();
    
    return hardwareReady && autoModeSelected;
  }
  
  // Helper methods for advanced condition checking
  
  /**
   * Checks if robot is inside alliance zone using vision.
   * Uses Vision pose data to determine if robot is in alliance scoring area.
   */
  private boolean isInAllianceZone() {
    // Get current Robot pose from drive system
    Pose2d currentPose = drive.getPose();
    
    // Check if we have recent vision data
    if (!vision.hasRecentVisionData(1.0)) {
      return false; // No recent vision data
    }
    
    // Get field dimensions from AprilTag layout
    double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
    double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
    
    // Define alliance zones (simplified - adjust based on your field layout)
    // Blue alliance: positive X side, Red alliance: negative X side
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    
    double allianceZoneXMin, allianceZoneXMax;
    double allianceZoneYMin, allianceZoneYMax;
    
    if (isBlueAlliance) {
      // Blue alliance zone (right side of field)
      allianceZoneXMin = fieldLength * 0.5; // Center line to right edge
      allianceZoneXMax = fieldLength * 0.9; // Near right edge
      allianceZoneYMin = fieldWidth * 0.2; // Bottom portion
      allianceZoneYMax = fieldWidth * 0.8; // Upper portion
    } else {
      // Red alliance zone (left side of field)
      allianceZoneXMin = fieldLength * 0.1; // Near left edge
      allianceZoneXMax = fieldLength * 0.5; // Center line to left edge
      allianceZoneYMin = fieldWidth * 0.2; // Bottom portion
      allianceZoneYMax = fieldWidth * 0.8; // Upper portion
    }
    
    // Check if robot is within alliance zone boundaries
    double robotX = currentPose.getX();
    double robotY = currentPose.getY();
    
    boolean inXZone = robotX >= allianceZoneXMin && robotX <= allianceZoneXMax;
    boolean inYZone = robotY >= allianceZoneYMin && robotY <= allianceZoneYMax;
    
    return inXZone && inYZone;
  }
  
  /**
   * Checks if flywheel is at the correct setpoint.
   */
  private boolean isFlywheelAtSetpoint() {
    // Check if drum velocity is within tolerance of target
    double currentRPM = drum.getAverageVelocityRPM();
    double targetRPM = shooterStateMachine.getTargetRPM();
    double tolerance = 50.0; // 50 RPM tolerance
    
    return Math.abs(currentRPM - targetRPM) < tolerance;
  }
  
  /**
   * Checks if hood is at the correct angle for shooting.
   */
  private boolean isHoodAtCorrectAngle() {
    // Check if hood angle is within tolerance of target
    double currentAngle = hood.getPositionDegrees();
    double targetAngle = shooterStateMachine.getTargetAngle();
    double tolerance = 2.0; // 2 degree tolerance
    
    return Math.abs(currentAngle - targetAngle) < tolerance;
  }
  
  /**
   * Checks if robot is aimed at the hub using vision/shooting controller.
   */
  private boolean isAimedAtHub() {
    // Use shooting controller's aim status
    return shooterStateMachine.isAiming();
  }
  
  /**
   * Checks if an auto mode is selected and ready.
   */
  private boolean isAutoModeSelected() {
    // Use hub activity as auto mode condition
    return robotContainer.isHubActive();
  }
  
  // Legacy getters for backward compatibility
  
  public RobotState getState() {
    return currentState;
  }
  
  public RobotState getCurrentState() {
    return currentState;
  }
  
  public RobotState getWantedState() {
    return wantedState;
  }
  
  /**
   * Sets shooting button held state.
   * Call this when shooting button is pressed.
   */
  public void wantToShoot(boolean held) {
    wantToShoot = held;
    Logger.recordOutput("MasterStateMachine/WantToShoot", held);
  }
  
  /**
   * Enables or disables auto-transition from aiming to shooting.
   * When enabled, holding shooting button in aiming state will auto-transition to shooting when ready.
   */
  public void setAutoShootingEnabled(boolean enabled) {
    autoShootingEnabled = enabled;
    Logger.recordOutput("MasterStateMachine/AutoShootingEnabled", enabled);
  }
  
  /**
   * Gets current shooting button held state.
   */
  public boolean wantingToShoot() {
    return wantToShoot;
  }
  
  /**
   * Gets current auto-shooting enabled state.
   */
  public boolean isAutoShootingEnabled() {
    return autoShootingEnabled;
  }
  
  /**
   * Sets hopper to compressing state for shooting preparation.
   * Call this when you want to compress game pieces before shooting.
   */
  public void compressHopper() {
    hopperStateMachine.setWantedState(HopperState.COMPRESSING);
  }
  
  /**
   * Gets current hopper state.
   */
  public HopperState getHopperState() {
    return hopperStateMachine.getCurrentState();
  }
  
  /**
   * Gets current shooter state.
   */
  public ShooterState getShooterState() {
    return shooterStateMachine.getCurrentState();
  }

  /**
   * Gets the shooter state machine.
   */
  public ShooterStateMachine getShooterStateMachine() {
    return shooterStateMachine;
  }

  /**
   * Gets the hopper state machine.
   */
  public HopperStateMachine getHopperStateMachine() {
    return hopperStateMachine;
  }

  // Status getters

  @AutoLogOutput
  public boolean isAiming() {
    return shooterStateMachine.isAiming();
  }

  @AutoLogOutput
  public boolean isReadyToShoot() {
    return shooterStateMachine.isReadyToShoot();
  }

  @AutoLogOutput
  public boolean isIntakeExtended() {
    return hopperStateMachine.isIntakeExtended();
  }

  @AutoLogOutput
  public double getTargetRPM() {
    return shooterStateMachine.getTargetRPM();
  }

  @AutoLogOutput
  public double getTargetAngle() {
    return shooterStateMachine.getTargetAngle();
  }

  @AutoLogOutput
  public double getCompensatedDistance() {
    return shooterStateMachine.getCompensatedDistance();
  }

  /**
   * Updates LED patterns based on current robot state.
   */
  private void updateLEDs() {
    long currentTime = System.currentTimeMillis();
    
    switch (currentState) {
      case IDLE:
        leds.setSolidColor(LEDPatterns.BLUE[0], LEDPatterns.BLUE[1], LEDPatterns.BLUE[2]);
        break;
        
      case INTAKING:
        // Red flash pattern for intaking
        boolean flashOn = ((currentTime / 100) % 2) == 0;
        double[] intakePattern = LEDPatterns.flashPattern(
            LEDPatterns.RED, LEDPatterns.LED_COUNT, flashOn);
        leds.setPattern(intakePattern, LEDPatterns.LED_COUNT);
        break;
        
      case AIMING:
        // Yellow breathing pattern for aiming
        double[] aimingPattern = LEDPatterns.breathingPattern(
            LEDPatterns.YELLOW, LEDPatterns.LED_COUNT,
            (currentTime / 1000.0) % (2 * Math.PI));
        leds.setPattern(aimingPattern, LEDPatterns.LED_COUNT);
        break;
        
      case SHOOTING:
        // Green chase pattern for shooting
        double[] shootingPattern = LEDPatterns.chasePattern(
            LEDPatterns.GREEN, LEDPatterns.LED_COUNT, 3, 
            (currentTime / 1000.0) % 1.0);
        leds.setPattern(shootingPattern, LEDPatterns.LED_COUNT);
        break;
        
      case FERRYING:
        // Purple solid color for ferrying
        leds.setSolidColor(LEDPatterns.PURPLE[0], LEDPatterns.PURPLE[1], LEDPatterns.PURPLE[2]);
        break;
        
      case DUMPING:
        // Orange chase pattern for dumping
        double[] dumpingPattern = LEDPatterns.chasePattern(
            LEDPatterns.ORANGE, LEDPatterns.LED_COUNT, 5,
            (currentTime / 500.0) % 1.0);
        leds.setPattern(dumpingPattern, LEDPatterns.LED_COUNT);
        break;
        
      case HOME:
        // Cyan solid color for homing
        leds.setSolidColor(LEDPatterns.CYAN[0], LEDPatterns.CYAN[1], LEDPatterns.CYAN[2]);
        break;
        
      case AUTO:
        // Rainbow pattern for autonomous
        double[] autoPattern = LEDPatterns.rainbowPattern(
            LEDPatterns.LED_COUNT, (currentTime / 2000.0) % 1.0);
        leds.setPattern(autoPattern, LEDPatterns.LED_COUNT);
        break;
        
      default:
        leds.setOff();
        break;
    }
  }
}
