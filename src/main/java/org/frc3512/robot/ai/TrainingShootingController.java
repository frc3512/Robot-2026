package org.frc3512.robot.ai;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.hopper.conveyor.Conveyor;
import org.frc3512.robot.subsystems.hopper.intake.Intake;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Training-focused shooting controller for AI data collection and model testing.
 * Separates training functionality from competition shooting for cleaner data.
 */
@SuppressWarnings("unused")
public class TrainingShootingController extends SubsystemBase {
  // Components
  private final Drive drive;
  private final Drum drum;
  private final Hood hood;
  private final Feeder feeder;
  private final Conveyor conveyor;
  private final Intake intake;
  
  // AI components
  private final AIShootingController aiController;
  private final TrainingDataCollector dataCollector;
  
  // Training state
  private boolean trainingMode = false;
  private final Timer shotTimer = new Timer();
  private int shotsInSequence = 0;
  private double sequenceStartTime = 0.0;
  
  // Test patterns
  public enum TestPattern {
    DISTANCE_SWEEP("Distance Sweep"),
    ANGLE_SWEEP("Angle Sweep"), 
    VELOCITY_TEST("Velocity Test"),
    RANDOM_SHOTS("Random Shots");
    
    private final String displayName;
    TestPattern(String displayName) {
      this.displayName = displayName;
    }
    
    public String getDisplayName() {
      return displayName;
    }
  }
  
  private TestPattern currentPattern = TestPattern.DISTANCE_SWEEP;
  private double testDistance = 3.0; // meters
  private double testAngle = 15.0; // degrees
  private double testVelocity = 0.0; // m/s

  public TrainingShootingController(
      Drive drive,
      Drum drum,
      Hood hood,
      Feeder feeder,
      Conveyor conveyor,
      Intake intake,
      AIShootingController aiController) {
    this.drive = drive;
    this.drum = drum;
    this.hood = hood;
    this.feeder = feeder;
    this.conveyor = conveyor;
    this.intake = intake;
    this.aiController = aiController;
    this.dataCollector = new TrainingDataCollector();
  }

  @Override
  public void periodic() {
    dataCollector.periodic();
    
    Logger.recordOutput("TrainingShootingController/TrainingMode", trainingMode);
    Logger.recordOutput("TrainingShootingController/CurrentPattern", currentPattern.getDisplayName());
    Logger.recordOutput("TrainingShootingController/ShotsInSequence", shotsInSequence);
    Logger.recordOutput("TrainingShootingController/SequenceTime", shotTimer.get());
  }

  /**
   * Starts training mode for systematic data collection.
   */
  public void startTrainingMode() {
    trainingMode = true;
    dataCollector.startCollection();
    aiController.startTrainingMode();
    Logger.recordOutput("TrainingShootingController/Status", "Training mode started");
  }

  /**
   * Stops training mode and returns to normal operation.
   */
  public void stopTrainingMode() {
    trainingMode = false;
    dataCollector.stopCollection();
    aiController.stopTrainingMode();
    Logger.recordOutput("TrainingShootingController/Status", "Training mode stopped");
  }

  /**
   * Sets the test pattern for systematic data collection.
   */
  public void setTestPattern(TestPattern pattern) {
    currentPattern = pattern;
    Logger.recordOutput("TrainingShootingController/TestPattern", pattern.getDisplayName());
  }

  /**
   * Executes a single training shot with current test parameters.
   */
  public void executeTrainingShot() {
    if (!trainingMode) {
      Logger.recordOutput("TrainingShootingController/Error", "Not in training mode");
      return;
    }
    
    // Get AI prediction for current test conditions
    Translation2d robotVelocity = getRobotVelocity();
    AIShootingController.ShootingParameters aiParams = 
        aiController.getShootingParameters(testDistance, robotVelocity, true);
    
    // Apply AI parameters
    drum.setVelocity(aiParams.rpm);
    hood.setAngle(aiParams.hoodAngle);
    
    // Start shooting sequence
    startShootingSequence();
    
    Logger.recordOutput("TrainingShootingController/ShotTaken", true);
    Logger.recordOutput("TrainingShootingController/TestDistance", testDistance);
    Logger.recordOutput("TrainingShootingController/TestAngle", testAngle);
    Logger.recordOutput("TrainingShootingController/AIRPM", aiParams.rpm);
    Logger.recordOutput("TrainingShootingController/AIAngle", aiParams.hoodAngle);
  }

  /**
   * Executes a systematic test pattern.
   */
  public void executeTestPattern() {
    switch (currentPattern) {
      case DISTANCE_SWEEP:
        executeDistanceSweep();
        break;
      case ANGLE_SWEEP:
        executeAngleSweep();
        break;
      case VELOCITY_TEST:
        executeVelocityTest();
        break;
      case RANDOM_SHOTS:
        executeRandomShots();
        break;
    }
  }

  /**
   * Distance sweep: Shoot from 1.5m to 6.0m in 0.5m increments.
   */
  private void executeDistanceSweep() {
    Logger.recordOutput("TrainingShootingController/Pattern", "Distance sweep started");
    
    for (double distance = 1.5; distance <= 6.0; distance += 0.5) {
      testDistance = distance;
      executeTrainingShot();
      Timer.delay(2.0); // Wait between shots
    }
  }

  /**
   * Angle sweep: Test different hood angles at fixed distance.
   */
  private void executeAngleSweep() {
    Logger.recordOutput("TrainingShootingController/Pattern", "Angle sweep started");
    
    testDistance = 3.0; // Fixed distance for angle test
    
    for (double angle = 5.0; angle <= 35.0; angle += 5.0) {
      testAngle = angle;
      executeTrainingShot();
      Timer.delay(1.5); // Wait between shots
    }
  }

  /**
   * Velocity test: Shoot while moving at different speeds.
   */
  private void executeVelocityTest() {
    Logger.recordOutput("TrainingShootingController/Pattern", "Velocity test started");
    
    testDistance = 4.0; // Fixed distance for velocity test
    
    // Test different robot velocities
    double[][] velocities = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}};
    
    for (double[] vel : velocities) {
      // Simulate robot motion by setting drive velocities
      drive.runVelocity(new ChassisSpeeds(vel[0], vel[1], 0.0));
      Timer.delay(0.5); // Let robot reach velocity
      
      testVelocity = Math.sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
      executeTrainingShot();
      Timer.delay(2.0); // Wait between shots
    }
    
    // Stop robot motion
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  /**
   * Random shots: Shoot from random positions and angles.
   */
  private void executeRandomShots() {
    Logger.recordOutput("TrainingShootingController/Pattern", "Random shots started");
    
    for (int i = 0; i < 20; i++) {
      // Random distance between 2.0m and 5.0m
      testDistance = 2.0 + (Math.random() * 3.0);
      
      // Random angle between 10° and 25°
      testAngle = 10.0 + (Math.random() * 15.0);
      
      // Random small velocity
      testVelocity = Math.random() * 0.5;
      
      executeTrainingShot();
      Timer.delay(1.0 + Math.random() * 2.0); // Random delay
    }
  }

  /**
   * Starts the shooting sequence timing.
   */
  private void startShootingSequence() {
    shotsInSequence++;
    if (shotsInSequence == 1) {
      sequenceStartTime = shotTimer.get();
    }
    shotTimer.restart();
  }

  /**
   * Records the result of a training shot.
   * Call this method when you know if the shot was successful.
   */
  public void recordShotResult(boolean successful, double accuracyError) {
    if (!trainingMode) return;
    
    double shotTime = shotTimer.get();
    Translation2d robotVelocity = getRobotVelocity();
    
    // Record with AI controller for learning
    aiController.recordShotResult(testDistance, robotVelocity, 
        new AIShootingController.ShootingParameters(
            drum.getAverageVelocityRPM(), hood.getPositionDegrees()), 
        successful, accuracyError);
    
    Logger.recordOutput("TrainingShootingController/ShotResult", successful);
    Logger.recordOutput("TrainingShootingController/AccuracyError", accuracyError);
    Logger.recordOutput("TrainingShootingController/ShotTime", shotTime);
    
    // Reset for next shot
    if (shotsInSequence >= 5) { // Reset sequence every 5 shots
      shotsInSequence = 0;
      Logger.recordOutput("TrainingShootingController/SequenceReset", true);
    }
  }

  /**
   * Gets current robot velocity from drive.
   */
  private Translation2d getRobotVelocity() {
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Trains the AI model with collected data.
   */
  public void trainAIModel() {
    Logger.recordOutput("TrainingShootingController/Training", "Starting AI training");
    
    TrainingPipeline pipeline = new TrainingPipeline(aiController.getNetwork());
    TrainingPipeline.TrainingResult result = pipeline.trainNetwork();
    
    if (result.success) {
      Logger.recordOutput("TrainingShootingController/TrainingSuccess", true);
      Logger.recordOutput("TrainingShootingController/TrainingLoss", result.validationLoss);
    } else {
      Logger.recordOutput("TrainingShootingController/TrainingError", result.message);
    }
  }

  /**
   * Gets training status and statistics.
   */
  public String getTrainingStatus() {
    if (!trainingMode) {
      return "Not in training mode";
    }
    
    int dataPoints = dataCollector.getBufferSize();
    String pattern = currentPattern.getDisplayName();
    
    return String.format("Training: %s | Data Points: %d | Pattern: %s", 
                       trainingMode ? "Active" : "Inactive", dataPoints, pattern);
  }

  /**
   * Gets AI controller for external access.
   */
  public AIShootingController getAIController() {
    return aiController;
  }

  /**
   * Gets data collector for external access.
   */
  public TrainingDataCollector getDataCollector() {
    return dataCollector;
  }

  /**
   * Stops all shooting operations safely.
   */
  public void emergencyStop() {
    drum.stop();
    hood.stop();
    feeder.stop();
    conveyor.stop();
    if (intake != null) {
      intake.stop();
    }
    
    Logger.recordOutput("TrainingShootingController/EmergencyStop", true);
  }
}
