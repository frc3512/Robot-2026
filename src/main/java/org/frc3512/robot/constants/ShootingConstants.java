package org.frc3512.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Shooting constants for the robot.
 * Includes PID gains, physics parameters, compensation gains, and interpolation tables.
 */
public final class ShootingConstants {
  
  // Aiming PID controller constants
  public static final double ANGLE_KP = 18.0;
  public static final double ANGLE_KD = 0.0;
  public static final double ANGLE_MAX_VELOCITY = 10.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;
  
  // Jitter compensation - suppress micro-corrections when within tolerance
  public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.5);
  
  // Tunable angle offset for clockwise rotation (degrees)
  public static final double ANGLE_OFFSET_DEGREES = 0.0;
  
  // Physics parameters for dynamic ball velocity calculation
  public static final double FLYWHEEL_RADIUS_M = 0.0508; // 4" diameter flywheel
  public static final double FRICTION_COEFFICIENT = 0.55; // μ rubber/foam
  public static final double COMPRESSION_FACTOR = 1.15; // Grip enhancement
  
  // Gain multipliers for compensation tuning
  public static final double LATERAL_COMPENSATION_GAIN = 3.75;
  public static final double RADIAL_COMPENSATION_GAIN = 0.05;
  
  // Distance clamping to stay within interpolation table range
  public static final double MIN_SHOT_DISTANCE = 1.25;
  public static final double MAX_SHOT_DISTANCE = 6.33;
  
  // Low-pass filter time constants (seconds)
  public static final double OMEGA_FILTER_TC = 0.06;
  public static final double DISTANCE_FILTER_TC = 0.10;
  
  // Latency compensation constant (seconds)
  public static final double LATENCY_SECONDS = 0.15;
  
  // Shooting sequence timing
  public static final double FEEDING_START_DELAY = 0.0; // Start feeding immediately when aimed
  public static final double INTAKE_AGITATION_START = 0.25; // Start agitation 0.25s after feeding
  public static final double INTAKE_AGITATION_PERIOD = 0.5; // Toggle every 0.5s
  
  // Shooting sequence motor outputs (RPM format)
  public static final double FEEDER_OUTPUT_RPM = 2800.0;
  public static final double CONVEYOR_OUTPUT_RPM = 2800.0;
  public static final double INTAKE_ROLLER_OUTPUT_RPM = 500.0;
  
  // Safe end values
  public static final double END_FLYWHEEL_RPM = 1800.0;
  public static final double END_HOOD_ANGLE = 10.0;
  
  // Interpolation tables for distance-based shooting
  public static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();
  
  // Profiled PID controller constraints
  public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION);
  
  static {
    // Initialize RPM table (DISTANCE (m) || RPM)
    RPM_TABLE.put(1.25, 2500.0);
    RPM_TABLE.put(1.88, 2800.0);
    RPM_TABLE.put(2.21, 2850.0);
    RPM_TABLE.put(3.07, 3100.0);
    RPM_TABLE.put(3.84, 3500.0);
    RPM_TABLE.put(4.11, 3550.0);
    RPM_TABLE.put(4.29, 3400.0);
    RPM_TABLE.put(4.40, 3500.0);
    RPM_TABLE.put(6.33, 3800.0);
    
    // Initialize angle table (DISTANCE (m) || HOOD ANGLE (degrees))
    ANGLE_TABLE.put(1.25, 7.0);
    ANGLE_TABLE.put(1.88, 9.0);
    ANGLE_TABLE.put(2.21, 11.0);
    ANGLE_TABLE.put(3.07, 13.0);
    ANGLE_TABLE.put(3.84, 11.0);
    ANGLE_TABLE.put(4.11, 12.0);
    ANGLE_TABLE.put(4.29, 15.0);
    ANGLE_TABLE.put(4.40, 15.0);
    ANGLE_TABLE.put(6.33, 20.0);
  }
  
  // Utility method to get RPM for distance
  public static double getRPMForDistance(double distance) {
    Double rpm = RPM_TABLE.get(distance);
    return rpm != null ? rpm : 2800.0; // Fallback value
  }
  
  // Utility method to get hood angle for distance
  public static double getAngleForDistance(double distance) {
    Double angle = ANGLE_TABLE.get(distance);
    return angle != null ? angle : 10.0; // Fallback value
  }
}
