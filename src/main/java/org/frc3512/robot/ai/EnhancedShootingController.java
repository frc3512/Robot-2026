package org.frc3512.robot.ai;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Enhanced shooting controller that integrates AI predictions with traditional methods.
 * Provides seamless fallback and switching between AI and lookup table approaches.
 */

@SuppressWarnings("unused")
public class EnhancedShootingController {
  // Components
  private final AIShootingController aiController;
  private final Drive drive;
  private final Drum drum;
  private final Hood hood;
  private final Feeder feeder;
  
  // Fallback control
  private enum ControlMode {
    TRADITIONAL_ONLY,
    AI_PRIMARY,
    AI_WITH_FALLBACK,
    AUTO_SWITCH
  }
  
  private ControlMode currentMode = ControlMode.AI_WITH_FALLBACK;
  private boolean aiEnabled = true;
  private boolean forceTraditional = false;
  private long lastModeSwitchTime = 0;
  private final long modeSwitchCooldownMs = 5000; // 5 seconds between switches
  
  // Performance tracking for auto-switching
  private int consecutiveAIFailures = 0;
  private int consecutiveTraditionalFailures = 0;
  private final int failureThreshold = 3;
  private double aiSuccessRate = 0.0;
  private double traditionalSuccessRate = 0.0;
  private final Timer performanceTimer = new Timer();
  private boolean lastShotSuccessful = false;
  
  // Shot tracking
  private double lastShotDistance = 0.0;
  private AIShootingController.ShootingParameters lastUsedParameters = null;
  private final Timer shotTimer = new Timer();

  public EnhancedShootingController(Drive drive, Drum drum, Hood hood, Feeder feeder) {
    this.drive = drive;
    this.drum = drum;
    this.hood = hood;
    this.feeder = feeder;
    this.aiController = new AIShootingController(drive, drum, hood, feeder);
    
    performanceTimer.start();
    shotTimer.start();
  }

  /**
   * Gets shooting parameters with intelligent fallback.
   */
  public AIShootingController.ShootingParameters getShootingParameters(
      double distanceToHub, Translation2d robotVelocity) {
    
    lastShotDistance = distanceToHub;
    
    switch (currentMode) {
      case TRADITIONAL_ONLY:
        return getTraditionalParameters(distanceToHub);
        
      case AI_PRIMARY:
        return getAIParameters(distanceToHub, robotVelocity, false);
        
      case AI_WITH_FALLBACK:
        return getAIWithFallback(distanceToHub, robotVelocity);
        
      case AUTO_SWITCH:
        return getAutoSwitchParameters(distanceToHub, robotVelocity);
        
      default:
        return getTraditionalParameters(distanceToHub);
    }
  }

  /**
   * Gets AI parameters with traditional fallback.
   */
  private AIShootingController.ShootingParameters getAIWithFallback(
      double distanceToHub, Translation2d robotVelocity) {
    
    if (!aiEnabled || forceTraditional) {
      return getTraditionalParameters(distanceToHub);
    }
    
    try {
      AIShootingController.ShootingParameters aiParams = 
          aiController.getShootingParameters(distanceToHub, robotVelocity, true);
      
      // Validate AI parameters
      if (validateParameters(aiParams)) {
        lastUsedParameters = aiParams;
        Logger.recordOutput("AI/EnhancedController/Method", "AI_Success");
        return aiParams;
      } else {
        Logger.recordOutput("AI/EnhancedController/Method", "AI_Fallback");
        consecutiveAIFailures++;
        return getTraditionalParameters(distanceToHub);
      }
    } catch (Exception e) {
      Logger.recordOutput("AI/EnhancedController/Error", "AI prediction failed: " + e.getMessage());
      consecutiveAIFailures++;
      return getTraditionalParameters(distanceToHub);
    }
  }

  /**
   * Automatically switches between AI and traditional based on performance.
   */
  private AIShootingController.ShootingParameters getAutoSwitchParameters(
      double distanceToHub, Translation2d robotVelocity) {
    
    // Determine which method to use based on recent performance
    boolean useAI = shouldUseAI();
    
    if (useAI) {
      try {
        AIShootingController.ShootingParameters aiParams = 
            aiController.getShootingParameters(distanceToHub, robotVelocity, true);
        
        if (validateParameters(aiParams)) {
          lastUsedParameters = aiParams;
          Logger.recordOutput("AI/EnhancedController/Method", "Auto_AI");
          return aiParams;
        }
      } catch (Exception e) {
        Logger.recordOutput("AI/EnhancedController/Error", "Auto AI failed: " + e.getMessage());
      }
    }
    
    // Fallback to traditional
    AIShootingController.ShootingParameters traditionalParams = getTraditionalParameters(distanceToHub);
    Logger.recordOutput("AI/EnhancedController/Method", "Auto_Traditional");
    return traditionalParams;
  }

  /**
   * Determines if AI should be used based on performance metrics.
   */
  private boolean shouldUseAI() {
    // If AI is disabled, don't use it
    if (!aiEnabled || forceTraditional) {
      return false;
    }
    
    // If AI has too many consecutive failures, use traditional
    if (consecutiveAIFailures >= failureThreshold) {
      return false;
    }
    
    // If traditional has too many consecutive failures, use AI
    if (consecutiveTraditionalFailures >= failureThreshold) {
      return true;
    }
    
    // Use success rates to decide
    double aiWeight = aiSuccessRate * 0.7 + (1.0 - consecutiveAIFailures / (double) failureThreshold) * 0.3;
    double traditionalWeight = traditionalSuccessRate * 0.7 + (1.0 - consecutiveTraditionalFailures / (double) failureThreshold) * 0.3;
    
    return aiWeight > traditionalWeight;
  }

  /**
   * Gets AI parameters (for AI_PRIMARY mode).
   */
  private AIShootingController.ShootingParameters getAIParameters(
      double distanceToHub, Translation2d robotVelocity, boolean allowFallback) {
    
    if (!aiEnabled) {
      return allowFallback ? getTraditionalParameters(distanceToHub) : 
                         new AIShootingController.ShootingParameters(2500.0, 10.0);
    }
    
    return aiController.getShootingParameters(distanceToHub, robotVelocity, allowFallback);
  }

  /**
   * Gets traditional parameters from lookup tables.
   */
  private AIShootingController.ShootingParameters getTraditionalParameters(double distanceToHub) {
    // Use existing ShootingConstants lookup
    double rpm = org.frc3512.robot.constants.ShootingConstants.getRPMForDistance(distanceToHub);
    double angle = org.frc3512.robot.constants.ShootingConstants.getAngleForDistance(distanceToHub);
    
    return new AIShootingController.ShootingParameters(rpm, angle);
  }

  /**
   * Validates shooting parameters are reasonable.
   */
  private boolean validateParameters(AIShootingController.ShootingParameters params) {
    // Check RPM bounds
    if (params.rpm < 1000.0 || params.rpm > 5000.0) {
      return false;
    }
    
    // Check hood angle bounds
    if (params.hoodAngle < 0.0 || params.hoodAngle > 45.0) {
      return false;
    }
    
    // Check for NaN or infinite values
    if (Double.isNaN(params.rpm) || Double.isInfinite(params.rpm) ||
        Double.isNaN(params.hoodAngle) || Double.isInfinite(params.hoodAngle)) {
      return false;
    }
    
    return true;
  }

  /**
   * Records shot result for performance tracking.
   */
  public void recordShotResult(boolean successful, double accuracyError) {
    lastShotSuccessful = successful;
    
    // Update performance counters
    if (lastUsedParameters != null) {
      boolean wasAI = currentMode == ControlMode.AI_PRIMARY || 
                     currentMode == ControlMode.AI_WITH_FALLBACK ||
                     (currentMode == ControlMode.AUTO_SWITCH && shouldUseAI());
      
      if (wasAI) {
        if (successful) {
          consecutiveAIFailures = 0;
        } else {
          consecutiveAIFailures++;
        }
      } else {
        if (successful) {
          consecutiveTraditionalFailures = 0;
        } else {
          consecutiveTraditionalFailures++;
        }
      }
    }
    
    // Record with AI controller
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();
    Translation2d robotVelocity = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    
    aiController.recordShotResult(lastShotDistance, robotVelocity, 
                              lastUsedParameters != null ? lastUsedParameters : 
                              getTraditionalParameters(lastShotDistance), 
                              successful, accuracyError);
    
    Logger.recordOutput("AI/EnhancedController/ConsecutiveAIFailures", consecutiveAIFailures);
    Logger.recordOutput("AI/EnhancedController/ConsecutiveTraditionalFailures", consecutiveTraditionalFailures);
  }

  /**
   * Sets the control mode.
   */
  public void setControlMode(String mode) {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastModeSwitchTime < modeSwitchCooldownMs) {
      Logger.recordOutput("AI/EnhancedController/Error", "Mode switch cooldown active");
      return;
    }
    
    try {
      ControlMode newMode = ControlMode.valueOf(mode.toUpperCase());
      if (newMode != currentMode) {
        currentMode = newMode;
        lastModeSwitchTime = currentTime;
        Logger.recordOutput("AI/EnhancedController/ControlMode", newMode.toString());
      }
    } catch (IllegalArgumentException e) {
      Logger.recordOutput("AI/EnhancedController/Error", "Invalid control mode: " + mode);
    }
  }

  /**
   * Enables/disables AI shooting.
   */
  public void setAIEnabled(boolean enabled) {
    this.aiEnabled = enabled;
    aiController.setAIEnabled(enabled);
    Logger.recordOutput("AI/EnhancedController/AIEnabled", enabled);
  }

  /**
   * Forces traditional mode temporarily.
   */
  public void setForceTraditional(boolean force) {
    this.forceTraditional = force;
    Logger.recordOutput("AI/EnhancedController/ForceTraditional", force);
  }

  /**
   * Starts AI training mode.
   */
  public void startTrainingMode() {
    aiController.startTrainingMode();
    Logger.recordOutput("AI/EnhancedController/TrainingMode", true);
  }

  /**
   * Stops AI training mode.
   */
  public void stopTrainingMode() {
    aiController.stopTrainingMode();
    Logger.recordOutput("AI/EnhancedController/TrainingMode", false);
  }

  /**
   * Trains the AI network.
   */
  public void trainNetwork() {
    TrainingPipeline pipeline = new TrainingPipeline(aiController.getNetwork());
    TrainingPipeline.TrainingResult result = pipeline.trainNetwork();
    
    Logger.recordOutput("AI/EnhancedController/TrainingResult", result.toString());
    
    if (result.success) {
      Logger.recordOutput("AI/EnhancedController/TrainingSuccess", true);
    } else {
      Logger.recordOutput("AI/EnhancedController/TrainingError", result.message);
    }
  }

  /**
   * Gets current status information.
   */
  public String getStatus() {
    return String.format("Mode: %s, AI: %s, AI Success: %.2f%%, Traditional Success: %.2f%%",
                      currentMode.toString(), aiEnabled ? "Enabled" : "Disabled",
                      aiSuccessRate * 100, traditionalSuccessRate * 100);
  }

  // Getters for subsystem access
  public AIShootingController getAIController() {
    return aiController;
  }

  public boolean isAIEnabled() {
    return aiEnabled;
  }

  public String getCurrentMode() {
    return currentMode.toString();
  }

  public int getConsecutiveAIFailures() {
    return consecutiveAIFailures;
  }

  public int getConsecutiveTraditionalFailures() {
    return consecutiveTraditionalFailures;
  }

  /**
   * Periodic method for logging and maintenance.
   */
  public void periodic() {
    aiController.periodic();
    
    // Update success rates periodically
    if (performanceTimer.get() > 10.0) { // Every 10 seconds
      aiSuccessRate = aiController.getSuccessRate();
      // Traditional success rate would need to be tracked separately
      performanceTimer.reset();
    }
    
    Logger.recordOutput("AI/EnhancedController/ControlMode", currentMode.toString());
    Logger.recordOutput("AI/EnhancedController/AIEnabled", aiEnabled);
    Logger.recordOutput("AI/EnhancedController/ForceTraditional", forceTraditional);
    Logger.recordOutput("AI/EnhancedController/AISuccessRate", aiSuccessRate);
    Logger.recordOutput("AI/EnhancedController/TraditionalSuccessRate", traditionalSuccessRate);
  }
}
