package org.frc3512.robot.ai;

import java.util.ArrayList;
import java.util.List;

import org.frc3512.robot.constants.ShootingConstants;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * AI-enhanced shooting controller that uses neural networks for parameter prediction.
 * Integrates with existing ShootingController architecture.
 */
@SuppressWarnings("unused")
public class AIShootingController {
  // Network architecture: 8 inputs -> 16 hidden -> 2 outputs
  private static final int INPUT_SIZE = 8;
  private static final int HIDDEN_SIZE = 16;
  private static final int OUTPUT_SIZE = 2;
  
  // Input indices
  private static final int IDX_DISTANCE = 0;
  private static final int IDX_VELOCITY_X = 1;
  private static final int IDX_VELOCITY_Y = 2;
  private static final int IDX_RADIAL_VEL = 3;
  private static final int IDX_TANGENTIAL_VEL = 4;
  private static final int IDX_BATTERY_VOLTAGE = 5;
  private static final int IDX_DRUM_TEMP = 6;
  private static final int IDX_CURRENT_HOOD_ANGLE = 7;
  
  // Output indices
  private static final int IDX_TARGET_RPM = 0;
  private static final int IDX_TARGET_HOOD_ANGLE = 1;
  
  // Components
  private final ShootingNeuralNetwork network;
  private final TrainingDataCollector dataCollector;
  private final Drive drive;
  private final Drum drum;
  private final Hood hood;
  private final Feeder feeder;
  
  // AI control state
  private boolean aiEnabled = false;
  private boolean trainingMode = false;
  private boolean collectingData = false;
  private double confidenceThreshold = 0.8;
  private long lastPredictionTime = 0;
  private double lastPredictionRPM = 0.0;
  private double lastPredictionAngle = 0.0;
  
  // Performance tracking
  private int totalPredictions = 0;
  private int successfulShots = 0;
  private double averageError = 0.0;
  private final List<Double> errorHistory = new ArrayList<>();
  private static final int MAX_ERROR_HISTORY = 100;

  public AIShootingController(Drive drive, Drum drum, Hood hood, Feeder feeder) {
    this.drive = drive;
    this.drum = drum;
    this.hood = hood;
    this.feeder = feeder;
    
    // Initialize AI components
    this.network = new ShootingNeuralNetwork(INPUT_SIZE, HIDDEN_SIZE, OUTPUT_SIZE);
    this.dataCollector = new TrainingDataCollector();
    
    // Try to load existing model
    boolean modelLoaded = network.loadModel();
    if (modelLoaded) {
      aiEnabled = true;
      Logger.recordOutput("AI/ShootingController/Status", "Model loaded and AI enabled");
    } else {
      Logger.recordOutput("AI/ShootingController/Status", "No model found, using traditional method");
    }
  }

  /**
   * Gets shooting parameters using AI prediction or traditional method.
   */
  public ShootingParameters getShootingParameters(
      double distanceToHub, Translation2d robotVelocity, boolean useAIFallback) {
    
    ShootingParameters params;
    
    if (aiEnabled && distanceToHub >= ShootingConstants.MIN_SHOT_DISTANCE && 
        distanceToHub <= ShootingConstants.MAX_SHOT_DISTANCE) {
      
      // Try AI prediction
      params = predictWithAI(distanceToHub, robotVelocity);
      
      // Check if prediction is reasonable
      if (isPredictionReasonable(params)) {
        lastPredictionTime = System.currentTimeMillis();
        lastPredictionRPM = params.rpm;
        lastPredictionAngle = params.hoodAngle;
        totalPredictions++;
        
        Logger.recordOutput("AI/ShootingController/Method", "AI");
        Logger.recordOutput("AI/ShootingController/Confidence", calculateConfidence(params));
        
        return params;
      } else if (useAIFallback) {
        Logger.recordOutput("AI/ShootingController/Method", "AI_Fallback");
      }
    }
    
    // Fallback to traditional method
    params = getTraditionalParameters(distanceToHub);
    Logger.recordOutput("AI/ShootingController/Method", "Traditional");
    
    return params;
  }

  /**
   * Predicts shooting parameters using neural network.
   */
  private ShootingParameters predictWithAI(double distanceToHub, Translation2d robotVelocity) {
    // Prepare input features
    double[] inputs = new double[INPUT_SIZE];
    inputs[IDX_DISTANCE] = distanceToHub;
    inputs[IDX_VELOCITY_X] = robotVelocity.getX();
    inputs[IDX_VELOCITY_Y] = robotVelocity.getY();
    
    // Calculate radial and tangential components
    Translation2d radialUnit = distanceToHub > 1e-6 ? 
        new Translation2d(distanceToHub, 0).div(distanceToHub) : Translation2d.kZero;
    Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());
    
    inputs[IDX_RADIAL_VEL] = robotVelocity.getX() * radialUnit.getX() + 
                            robotVelocity.getY() * radialUnit.getY();
    inputs[IDX_TANGENTIAL_VEL] = robotVelocity.getX() * tangentialUnit.getX() + 
                                robotVelocity.getY() * tangentialUnit.getY();
    
    // Get system state
    inputs[IDX_BATTERY_VOLTAGE] = getBatteryVoltage();
    inputs[IDX_DRUM_TEMP] = drum.getAverageTempCelsius();
    inputs[IDX_CURRENT_HOOD_ANGLE] = hood.getPositionDegrees();
    
    // Get prediction
    double[] outputs = network.predict(inputs);
    
    return new ShootingParameters(outputs[IDX_TARGET_RPM], outputs[IDX_TARGET_HOOD_ANGLE]);
  }

  /**
   * Gets traditional shooting parameters from lookup tables.
   */
  private ShootingParameters getTraditionalParameters(double distanceToHub) {
    double rpm = ShootingConstants.getRPMForDistance(distanceToHub);
    double angle = ShootingConstants.getAngleForDistance(distanceToHub);
    return new ShootingParameters(rpm, angle);
  }

  /**
   * Checks if AI prediction is reasonable.
   */
  private boolean isPredictionReasonable(ShootingParameters params) {
    // Check RPM bounds
    if (params.rpm < 1000.0 || params.rpm > 5000.0) {
      return false;
    }
    
    // Check hood angle bounds
    if (params.hoodAngle < 0.0 || params.hoodAngle > 45.0) {
      return false;
    }
    
    // Check if prediction is too far from traditional values
    double traditionalRPM = ShootingConstants.getRPMForDistance(
        ShootingConstants.MIN_SHOT_DISTANCE + (ShootingConstants.MAX_SHOT_DISTANCE - ShootingConstants.MIN_SHOT_DISTANCE) / 2.0);
    double rpmDiff = Math.abs(params.rpm - traditionalRPM);
    
    if (rpmDiff > traditionalRPM * 0.5) { // More than 50% difference
      return false;
    }
    
    return true;
  }

  /**
   * Calculates confidence in AI prediction.
   */
  private double calculateConfidence(ShootingParameters params) {
    // Simple confidence calculation based on parameter reasonableness
    double confidence = 1.0;
    
    // Reduce confidence for extreme values
    if (params.rpm < 1500.0 || params.rpm > 4000.0) {
      confidence *= 0.8;
    }
    
    if (params.hoodAngle < 5.0 || params.hoodAngle > 30.0) {
      confidence *= 0.8;
    }
    
    // Reduce confidence if network hasn't been updated recently
    long timeSinceUpdate = System.currentTimeMillis() - network.getLastUpdateTime();
    if (timeSinceUpdate > 24 * 60 * 60 * 1000) { // More than 24 hours old
      confidence *= 0.7;
    }
    
    return confidence;
  }

  /**
   * Records shot result for training.
   */
  public void recordShotResult(
      double distanceToHub, Translation2d robotVelocity, 
      ShootingParameters usedParams, boolean successful, double accuracyError) {
    
    if (!collectingData) return;
    
    // Create training data point
    TrainingDataCollector.ShootingDataPoint dataPoint = 
        new TrainingDataCollector.ShootingDataPoint(
            distanceToHub, robotVelocity.getX(), robotVelocity.getY(),
            calculateRadialVelocity(distanceToHub, robotVelocity),
            calculateTangentialVelocity(distanceToHub, robotVelocity),
            getBatteryVoltage(), drum.getAverageTempCelsius(), hood.getPositionDegrees(),
            System.currentTimeMillis(),
            usedParams.rpm, usedParams.hoodAngle, successful, accuracyError, accuracyError
        );
    
    dataCollector.addDataPoint(dataPoint);
    
    // Update performance tracking
    if (successful) {
      successfulShots++;
    }
    
    errorHistory.add(accuracyError);
    if (errorHistory.size() > MAX_ERROR_HISTORY) {
      errorHistory.remove(0);
    }
    
    averageError = errorHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    
    Logger.recordOutput("AI/ShootingController/SuccessRate", 
                       totalPredictions > 0 ? (double) successfulShots / totalPredictions : 0.0);
    Logger.recordOutput("AI/ShootingController/AverageError", averageError);
  }

  private double calculateRadialVelocity(double distanceToHub, Translation2d robotVelocity) {
    if (distanceToHub <= 1e-6) return 0.0;
    Translation2d radialUnit = new Translation2d(distanceToHub, 0).div(distanceToHub);
    return robotVelocity.getX() * radialUnit.getX() + robotVelocity.getY() * radialUnit.getY();
  }

  private double calculateTangentialVelocity(double distanceToHub, Translation2d robotVelocity) {
    if (distanceToHub <= 1e-6) return 0.0;
    Translation2d radialUnit = new Translation2d(distanceToHub, 0).div(distanceToHub);
    Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());
    return robotVelocity.getX() * tangentialUnit.getX() + robotVelocity.getY() * tangentialUnit.getY();
  }

  private double getBatteryVoltage() {
    // This would typically come from the power distribution panel
    // For now, return a nominal value
    return 12.5;
  }

  /**
   * Starts training mode.
   */
  public void startTrainingMode() {
    trainingMode = true;
    collectingData = true;
    dataCollector.startCollection();
    Logger.recordOutput("AI/ShootingController/TrainingMode", true);
  }

  /**
   * Stops training mode.
   */
  public void stopTrainingMode() {
    trainingMode = false;
    collectingData = false;
    dataCollector.stopCollection();
    Logger.recordOutput("AI/ShootingController/TrainingMode", false);
  }

  /**
   * Trains the network with collected data.
   */
  public void trainNetwork() {
    List<String> dataFiles = dataCollector.getDataFiles();
    if (dataFiles.isEmpty()) {
      Logger.recordOutput("AI/ShootingController/Error", "No training data available");
      return;
    }
    
    // This would typically load and process the CSV files
    // For now, we'll just log that training would happen
    Logger.recordOutput("AI/ShootingController/Training", "Training network with " + dataFiles.size() + " files");
    
    // Save the trained model
    network.saveModel();
  }

  /**
   * Enables/disables AI shooting.
   */
  public void setAIEnabled(boolean enabled) {
    this.aiEnabled = enabled && network.isInitialized();
    Logger.recordOutput("AI/ShootingController/AIEnabled", this.aiEnabled);
  }

  /**
   * Gets AI status.
   */
  public boolean isAIEnabled() {
    return aiEnabled;
  }

  public boolean isTrainingMode() {
    return trainingMode;
  }

  public boolean isCollectingData() {
    return collectingData;
  }

  /**
   * Gets the neural network for training.
   */
  public ShootingNeuralNetwork getNetwork() {
    return network;
  }

  /**
   * Gets performance statistics.
   */
  public int getTotalPredictions() {
    return totalPredictions;
  }

  public int getSuccessfulShots() {
    return successfulShots;
  }

  public double getSuccessRate() {
    return totalPredictions > 0 ? (double) successfulShots / totalPredictions : 0.0;
  }

  public double getAverageError() {
    return averageError;
  }

  /**
   * Periodic method for logging and maintenance.
   */
  public void periodic() {
    dataCollector.periodic();
    
    Logger.recordOutput("AI/ShootingController/AIEnabled", aiEnabled);
    Logger.recordOutput("AI/ShootingController/NetworkInitialized", network.isInitialized());
    Logger.recordOutput("AI/ShootingController/TotalPredictions", totalPredictions);
    Logger.recordOutput("AI/ShootingController/SuccessfulShots", successfulShots);
    Logger.recordOutput("AI/ShootingController/SuccessRate", getSuccessRate());
    Logger.recordOutput("AI/ShootingController/AverageError", averageError);
    Logger.recordOutput("AI/ShootingController/LastPredictionRPM", lastPredictionRPM);
    Logger.recordOutput("AI/ShootingController/LastPredictionAngle", lastPredictionAngle);
  }

  /**
   * Data structure for shooting parameters.
   */
  public static class ShootingParameters {
    public final double rpm;
    public final double hoodAngle;
    
    public ShootingParameters(double rpm, double hoodAngle) {
      this.rpm = rpm;
      this.hoodAngle = hoodAngle;
    }
    
    @Override
    public String toString() {
      return String.format("RPM: %.1f, Angle: %.1f°", rpm, hoodAngle);
    }
  }
}
