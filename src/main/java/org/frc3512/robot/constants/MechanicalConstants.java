package org.frc3512.robot.constants;

/**
 * Mechanical constants for all subsystems.
 * Includes gear ratios, dimensions, and conversion factors.
 */
public final class MechanicalConstants {
  
  // Intake constants
  public static final class Intake {
    // Linear slide positions in inches
    public static final double EXTENDED_POSITION_INCHES = 12.0; // TODO: Tune this value
    public static final double RETRACTED_POSITION_INCHES = 0.0;
    
    // Roller conversion factors (mechanism RPM to motor RPM)
    public static final double ROLLER_GEAR_RATIO = 1.0; // TODO: Update with actual gear ratio
    public static final double ROLLER_DIAMETER_INCHES = 2.0; // TODO: Update with actual diameter
    
    // Conversion: mechanism RPM to motor RPM
    public static final double ROLLER_MOTOR_RPM_PER_MECHANISM_RPM = ROLLER_GEAR_RATIO;
    
    // Conversion: mechanism RPM to motor velocity units (rotations per second)
    public static final double ROLLER_MOTOR_RPS_PER_MECHANISM_RPM = ROLLER_GEAR_RATIO / 60.0;
  }
  
  // Conveyor constants
  public static final class Conveyor {
    // Roller conversion factors (mechanism RPM to motor RPM)
    public static final double ROLLER_GEAR_RATIO = 1.0; // TODO: Update with actual gear ratio
    public static final double ROLLER_DIAMETER_INCHES = 2.0; // TODO: Update with actual diameter
    
    // Conversion: mechanism RPM to motor RPM
    public static final double ROLLER_MOTOR_RPM_PER_MECHANISM_RPM = ROLLER_GEAR_RATIO;
    
    // Conversion: mechanism RPM to motor velocity units (rotations per second)
    public static final double ROLLER_MOTOR_RPS_PER_MECHANISM_RPM = ROLLER_GEAR_RATIO / 60.0;
  }
  
  // Feeder constants
  public static final class Feeder {
    // Feed mechanism conversion factors (mechanism RPM to motor RPM)
    public static final double FEED_GEAR_RATIO = 1.0; // TODO: Update with actual gear ratio
    public static final double FEED_WHEEL_DIAMETER_INCHES = 2.0; // TODO: Update with actual diameter
    
    // Conversion: mechanism RPM to motor RPM
    public static final double FEED_MOTOR_RPM_PER_MECHANISM_RPM = FEED_GEAR_RATIO;
    
    // Conversion: mechanism RPM to motor velocity units (rotations per second)
    public static final double FEED_MOTOR_RPS_PER_MECHANISM_RPM = FEED_GEAR_RATIO / 60.0;
  }
  
  // Hood constants
  public static final class Hood {
    // Angle limits in degrees
    public static final double MIN_ANGLE_DEGREES = 10.0;
    public static final double MAX_ANGLE_DEGREES = 45.0;
    
    // Conversion: degrees to rotations
    public static final double DEGREES_TO_ROTATIONS = 1.0 / 360.0;
    
    // Gear ratio for hood mechanism
    public static final double GEAR_RATIO = 164.7 / 1.0;
  }
  
  // Drum constants
  public static final class Drum {
    // Drum dimensions
    public static final double DRUM_RADIUS_INCHES = 2.0;
    public static final double DRUM_CIRCUMFERENCE_INCHES = 2.0 * Math.PI * DRUM_RADIUS_INCHES;
    
    // Gear ratio
    public static final double GEAR_RATIO = 1.0;
    
    // Surface speed conversion: RPM to inches per second
    public static final double RPM_TO_INCHES_PER_SECOND = DRUM_CIRCUMFERENCE_INCHES / 60.0;
    
    // Surface speed conversion: RPS to inches per second
    public static final double RPS_TO_INCHES_PER_SECOND = DRUM_CIRCUMFERENCE_INCHES;
  }
}
