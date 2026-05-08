package org.frc3512.robot.ai;

/**
 * Configuration constants for AI shooting system.
 * Contains all tunable parameters for neural network, training, and control.
 */
public final class AIConstants {
  
  // Neural Network Architecture
  public static final int INPUT_SIZE = 8;
  public static final int HIDDEN_SIZE = 16;
  public static final int OUTPUT_SIZE = 2;
  
  // Network Training Parameters
  public static final double LEARNING_RATE = 0.01;
  public static final double REGULARIZATION = 0.001;
  public static final int MAX_EPOCHS = 100;
  public static final int BATCH_SIZE = 32;
  public static final double VALIDATION_SPLIT = 0.2;
  public static final double EARLY_STOPPING_PATIENCE = 10.0;
  
  // Data Collection Parameters
  public static final String DATA_DIR = "/tmp/robot_ai_data";
  public static final String MODEL_DIR = "/tmp/robot_ai_models";
  public static final String MODEL_FILE = "shooting_model.nn";
  public static final int MAX_BUFFER_SIZE = 1000;
  public static final long DATA_SAVE_INTERVAL_MS = 30000; // 30 seconds
  
  // AI Control Parameters
  public static final double CONFIDENCE_THRESHOLD = 0.8;
  public static final int FAILURE_THRESHOLD = 3;
  public static final long MODE_SWITCH_COOLDOWN_MS = 5000; // 5 seconds
  public static final double PERFORMANCE_UPDATE_INTERVAL_S = 10.0; // 10 seconds
  
  // Parameter Validation Bounds
  public static final double MIN_RPM = 1000.0;
  public static final double MAX_RPM = 5000.0;
  public static final double MIN_HOOD_ANGLE = 0.0;
  public static final double MAX_HOOD_ANGLE = 45.0;
  public static final double MAX_RPM_DIFFERENCE_RATIO = 0.5; // 50% difference from traditional
  
  // Training Data Filtering
  public static final double MAX_ACCURACY_ERROR = 0.5;
  public static final boolean ONLY_USE_SUCCESSFUL_SHOTS = true;
  public static final int MAX_ERROR_HISTORY = 100;
  
  // Feature Normalization Defaults
  public static final double[] DEFAULT_INPUT_MEAN = {
      3.0,    // distance_to_hub (m)
      0.0,    // robot_velocity_x (m/s)
      0.0,    // robot_velocity_y (m/s)
      0.0,    // radial_velocity (m/s)
      0.0,    // tangential_velocity (m/s)
      12.5,    // battery_voltage (V)
      40.0,    // drum_temperature (C)
      15.0     // hood_angle (degrees)
  };
  
  public static final double[] DEFAULT_INPUT_STD = {
      1.5,     // distance_to_hub (m)
      2.0,     // robot_velocity_x (m/s)
      2.0,     // robot_velocity_y (m/s)
      2.0,     // radial_velocity (m/s)
      2.0,     // tangential_velocity (m/s)
      1.0,     // battery_voltage (V)
      10.0,    // drum_temperature (C)
      10.0     // hood_angle (degrees)
  };
  
  public static final double[] DEFAULT_OUTPUT_MEAN = {
      2800.0,  // target_rpm
      15.0     // target_hood_angle
  };
  
  public static final double[] DEFAULT_OUTPUT_STD = {
      500.0,   // target_rpm
      5.0      // target_hood_angle
  };
  
  // Logging and Debugging
  public static final boolean ENABLE_DETAILED_LOGGING = true;
  public static final boolean ENABLE_PERFORMANCE_TRACKING = true;
  public static final boolean ENABLE_DATA_COLLECTION = true;
  
  // AI Mode Configuration
  public static enum ControlMode {
    TRADITIONAL_ONLY("Traditional Only"),
    AI_PRIMARY("AI Primary"),
    AI_WITH_FALLBACK("AI with Fallback"),
    AUTO_SWITCH("Auto Switch");
    
    private final String displayName;
    
    ControlMode(String displayName) {
      this.displayName = displayName;
    }
    
    public String getDisplayName() {
      return displayName;
    }
  }
  
  // Default Settings
  public static final ControlMode DEFAULT_CONTROL_MODE = ControlMode.AI_WITH_FALLBACK;
  public static final boolean DEFAULT_AI_ENABLED = true;
  public static final boolean DEFAULT_TRAINING_MODE = false;
  
  // Performance Metrics
  public static final double MIN_SUCCESS_RATE_FOR_AI = 0.7; // 70%
  public static final double MIN_SUCCESS_RATE_FOR_TRADITIONAL = 0.6; // 60%
  
  // Model Update Triggers
  public static final int MIN_DATA_POINTS_FOR_TRAINING = 50;
  public static final int TRAINING_INTERVAL_HOURS = 24;
  public static final double MIN_IMPROVEMENT_THRESHOLD = 0.05; // 5% improvement
  
  // Safety and Reliability
  public static final double MAX_PREDICTION_AGE_HOURS = 24.0;
  public static final boolean ENABLE_PREDICTION_VALIDATION = true;
  public static final boolean ENABLE_AUTO_FALLBACK = true;
  
  // System Integration
  public static final boolean INTEGRATE_WITH_EXISTING_CONTROLLER = true;
  public static final boolean PRESERVE_EXISTING_LOGIC = true;
  public static final boolean ENABLE_GRADUAL_ROLLOUT = true;
  
  // Private constructor to prevent instantiation
  private AIConstants() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
  
  // Utility methods for configuration validation
  public static boolean isValidRPM(double rpm) {
    return rpm >= MIN_RPM && rpm <= MAX_RPM && !Double.isNaN(rpm) && !Double.isInfinite(rpm);
  }
  
  public static boolean isValidHoodAngle(double angle) {
    return angle >= MIN_HOOD_ANGLE && angle <= MAX_HOOD_ANGLE && 
           !Double.isNaN(angle) && !Double.isInfinite(angle);
  }
  
  public static boolean isValidShootingParameters(double rpm, double hoodAngle) {
    return isValidRPM(rpm) && isValidHoodAngle(hoodAngle);
  }
  
  public static double clampRPM(double rpm) {
    return Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
  }
  
  public static double clampHoodAngle(double angle) {
    return Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, angle));
  }
}
