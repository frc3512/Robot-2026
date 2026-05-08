package org.frc3512.robot.constants;

import org.frc3512.robot.subsystems.states.RobotState;

/**
 * Constants for current limits across all subsystems and robot states.
 * These values are in amps and can be tuned post-code generation.
 */
public final class CurrentLimits {
  
  // Helper method to reduce code duplication across subsystems
  private static double getLimitForState(double[][] limits, RobotState state, int index) {
    return switch (state) {
      case IDLE -> limits[0][index];
      case INTAKING -> limits[1][index];
      case AIMING -> limits[2][index];
      case SHOOTING -> limits[3][index];
      case FERRYING -> limits[4][index];
      case DUMPING -> limits[5][index];
      case HOME -> limits[6][index];
      case AUTO -> limits[7][index];
    };
  }
  
  // Drive subsystem current limits (per motor)
  public static final class Drive {
    // [supply, stator] current limits for each state
    public static final double[][] LIMITS = {
      {40.0, 80.0}, // IDLE
      {80.0, 120.0}, // INTAKING  
      {20.0, 20.0}, // AIMING
      {15.0, 15.0}, // SHOOTING
      {20.0, 20.0}, // FERRYING
      {10.0, 10.0}, // DUMPING
      {20.0, 40.0}, // HOME
      {120.0, 120.0}  // AUTO
    };
    
    public static double getSupplyForState(RobotState state) {
      return getLimitForState(LIMITS, state, 0);
    }
    
    public static double getStatorForState(RobotState state) {
      return getLimitForState(LIMITS, state, 1);
    }
  }
  
  // Steer motor current limits (always constant)
  public static final class Steer {
    public static final double[] LIMITS = {20.0, 20.0}; // [supply, stator]
  }
  
  // Intake subsystem current limits (per motor)
  public static final class Intake {
    // [supply, stator] current limits for each state
    public static final double[][] LIMITS = {
      {10.0, 10.0},   // IDLE
      {40.0, 80.0},  // INTAKING
      {10.0, 20.0},    // AIMING
      {20.0, 20.0},   // SHOOTING
      {20.0, 20.0},   // FERRYING
      {20.0, 20.0},   // DUMPING
      {5.0, 5.0},     // HOME
      {120.0, 120.0}     // AUTO
    };
    
    public static double getSupplyForState(RobotState state) {
      return getLimitForState(LIMITS, state, 0);
    }
    
    public static double getStatorForState(RobotState state) {
      return getLimitForState(LIMITS, state, 1);
    }
  }
  
  // Conveyor subsystem current limits
  public static final class Conveyor {
    // [supply, stator] current limits for each state
    public static final double[][] LIMITS = {
      {10.0, 10.0},   // IDLE
      {20.0, 20.0},  // INTAKING
      {20.0, 20.0},    // AIMING
      {40.0, 30.0},  // SHOOTING
      {20.0, 20.0},  // FERRYING
      {15.0, 15.0},  // DUMPING
      {10.0, 10.0},    // HOME
      {120.0, 120.0}     // AUTO
    };
    
    public static double getSupplyForState(RobotState state) {
      return getLimitForState(LIMITS, state, 0);
    }
    
    public static double getStatorForState(RobotState state) {
      return getLimitForState(LIMITS, state, 1);
    }
  }
  
  // Feeder subsystem current limits (per motor)
  public static final class Feeder {
    // [supply, stator] current limits for each state
    public static final double[][] LIMITS = {
      {10.0, 10.0},   // IDLE
      {10.0, 10.0},   // INTAKING
      {15.0, 15.0},   // AIMING
      {40.0, 60.0},   // SHOOTING
      {30.0, 40.0},   // FERRYING
      {20.0, 20.0},   // DUMPING
      {10.0, 10.0},     // HOME
      {120.0, 120.0}     // AUTO
    };
    
    public static double getSupplyForState(RobotState state) {
      return getLimitForState(LIMITS, state, 0);
    }
    
    public static double getStatorForState(RobotState state) {
      return getLimitForState(LIMITS, state, 1);
    }
  }
  
  // Hood subsystem current limits (no state-based limits as requested)
  public static final class Hood {
    public static final double LIMIT = 20.0;
  }
  
  // Drum subsystem current limits (per motor)
  public static final class Drum {
    // [supply, stator] current limits for each state
    public static final double[][] LIMITS = {
      {20.0, 20.0},   // IDLE
      {20.0, 20.0},   // INTAKING
      {40.0, 80.0},   // AIMING
      {30.0, 80.0},   // SHOOTING
      {20.0, 40.0},   // FERRYING
      {20.0, 20.0},   // DUMPING
      {10.0, 10.0},   // HOME
      {120.0, 120.0}     // AUTO
    };
    
    public static double getSupplyForState(RobotState state) {
      return getLimitForState(LIMITS, state, 0);
    }
    
    public static double getStatorForState(RobotState state) {
      return getLimitForState(LIMITS, state, 1);
    }
  }
}
