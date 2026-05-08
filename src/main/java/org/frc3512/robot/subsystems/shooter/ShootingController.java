package org.frc3512.robot.subsystems.shooter;

import org.frc3512.robot.constants.ShootingConstants;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.hopper.conveyor.Conveyor;
import org.frc3512.robot.subsystems.hopper.intake.Intake;
import org.frc3512.robot.subsystems.hopper.HopperState;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.statemachine.MasterStateMachine;
import org.frc3512.robot.Constants;
import org.frc3512.robot.ai.EnhancedShootingController;
import org.frc3512.robot.ai.AIShootingController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Shooting controller that handles aiming, compensation, and shooting sequence.
 * Integrates the complex ShootAndMove logic into the state machine architecture.
 */
public class ShootingController {
  private final Drive drive;
  private final Drum drum;
  private final Hood hood;
  private final Conveyor conveyor;
  private final Feeder feeder;
  private final Intake intake;

  private final ProfiledPIDController angleController;

  private final EnhancedShootingController enhancedShootingController;
  
  // Low-pass filters
  private final LinearFilter omegaFilter = LinearFilter.singlePoleIIR(ShootingConstants.OMEGA_FILTER_TC, 0.02);
  private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(ShootingConstants.DISTANCE_FILTER_TC, 0.02);

  // Shooting sequence state
  private final Timer shootTimer = new Timer();
  private final Timer feedingTimer = new Timer();
  private boolean feedingStarted = false;
  private boolean lastIntakeExtended = false;
  
  // Shooting state
  private boolean active = false;
  private double targetRPM = 0.0;
  private double targetAngle = 0.0;
  private double compensatedDistance = 0.0;

  public ShootingController(
    Drive drive,
    Drum drum, 
    Hood hood,
    Conveyor conveyor,
    Feeder feeder,
    Intake intake,
    EnhancedShootingController enhancedController) {
    this.drive = drive;
    this.drum = drum;
    this.hood = hood;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.intake = intake;
    this.enhancedShootingController = enhancedController;
    
    angleController = new ProfiledPIDController(
        ShootingConstants.ANGLE_KP,
        0.0,
        ShootingConstants.ANGLE_KD,
        ShootingConstants.ANGLE_CONSTRAINTS);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructor for shooter state machine (no intake control).
   */
  public ShootingController(
      Drive drive,
      Drum drum,
      Hood hood,
      Feeder feeder,
      EnhancedShootingController enhancedController) {
    this(drive, drum, hood, null, feeder, null, enhancedController);
  }

  /**
   * Starts the shooting sequence.
   */
  public void start() {
    if (!active) {
      active = true;
      angleController.reset(drive.getRotation().getRadians());
      omegaFilter.reset();
      distanceFilter.reset();
      
      shootTimer.restart();
      feedingTimer.stop();
      feedingTimer.reset();
      feedingStarted = false;
      lastIntakeExtended = false;
      
      // Stop feeder and conveyor at start
      feeder.setFeedVelocity(0.0);
      feeder.setBoosterVelocity(0.0);
      conveyor.setVelocity(0.0);
      
      // Only control intake if available
      if (intake != null) {
        intake.setRollerVelocity(0.0);
      }
      
      Logger.recordOutput("ShootingController/Active", true);
    }
  }

  /**
   * Stops the shooting sequence.
   */
  public void stop() {
    if (active) {
      active = false;
      
      // Safe end values
      drum.setVelocity(ShootingConstants.END_FLYWHEEL_RPM);
      hood.setAngle(ShootingConstants.END_HOOD_ANGLE);
      
      // Stop feeder and conveyor
      feeder.setFeedVelocity(0.0);
      feeder.setBoosterVelocity(0.0);
      conveyor.setVelocity(0.0);
      
      // Only control intake if available
      if (intake != null) {
        intake.setRollerVelocity(0.0);
        intake.retract();
      }
      
      Logger.recordOutput("ShootingController/Active", false);
    }
  }

  /**
   * Updates the shooting controller. Should be called periodically when active.
   */
  public void update() {
    if (!active) return;

    // --- AIMING CALCULATION ---
    
    // Get current pose and speeds
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();

    // Convert robot-relative chassis speeds to field-relative velocity vector
    Translation2d robotVelocityField =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .rotateBy(drive.getRotation());

    // 1. LATENCY COMPENSATION
    Translation2d futurePos = robotPose.getTranslation().plus(robotVelocityField.times(ShootingConstants.LATENCY_SECONDS));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = Constants.FieldConstants.hubPosition();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double rawDistanceToHub = targetVec.getNorm();

    // 3. MOVING-WHILE-SHOOTING COMPENSATION
    double tFlight = calculateFlightTime(rawDistanceToHub);
    
    // Unit vectors in radial and tangential directions
    Translation2d radialUnit =
        rawDistanceToHub > 1e-6 ? targetVec.div(rawDistanceToHub) : Translation2d.kZero;
    Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

    // Signed velocity components
    double radialVelocity =
        robotVelocityField.getX() * radialUnit.getX()
            + robotVelocityField.getY() * radialUnit.getY();
    double tangentialVelocity =
        robotVelocityField.getX() * tangentialUnit.getX()
            + robotVelocityField.getY() * tangentialUnit.getY();

    // Angle lead from tangential component
    double lateralLeadDistance = -tangentialVelocity * tFlight * ShootingConstants.LATERAL_COMPENSATION_GAIN;
    double baseHeadingRadians = Math.atan2(targetVec.getY(), targetVec.getX());
    double compensatedHeadingRadians =
        baseHeadingRadians + Math.atan2(lateralLeadDistance, rawDistanceToHub);
    compensatedHeadingRadians += Math.toRadians(ShootingConstants.ANGLE_OFFSET_DEGREES);
    Rotation2d desiredHeading = new Rotation2d(compensatedHeadingRadians);

    // Radial compensation adjusts effective distance
    double radialDistanceOffset = radialVelocity * tFlight * ShootingConstants.RADIAL_COMPENSATION_GAIN;
    double compensatedDistanceRaw = rawDistanceToHub + radialDistanceOffset;
    double compensatedDistanceClamped = MathUtil.clamp(
        compensatedDistanceRaw, 
        ShootingConstants.MIN_SHOT_DISTANCE, 
        ShootingConstants.MAX_SHOT_DISTANCE);

    // Calculate angular error and apply deadband
    double headingError = MathUtil.angleModulus(desiredHeading.getRadians() - drive.getRotation().getRadians());
    double omegaRaw = 0.0;
    if (Math.abs(headingError) > ShootingConstants.ANGLE_TOLERANCE_RADIANS) {
      omegaRaw = angleController.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());
    }
    double omega = omegaFilter.calculate(omegaRaw);

    // Update drive for aiming
    boolean aimedAtHub = Math.abs(headingError) <= ShootingConstants.ANGLE_TOLERANCE_RADIANS;
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    // 4. SET FLYWHEEL RPM AND HOOD ANGLE
    double filteredDistanceToHub = distanceFilter.calculate(compensatedDistanceClamped);
    
    // Use AI shooting parameters with fallback to traditional
    if (enhancedShootingController != null) {
      AIShootingController.ShootingParameters aiParams = 
          enhancedShootingController.getShootingParameters(filteredDistanceToHub, robotVelocityField);
      targetRPM = aiParams.rpm;
      targetAngle = aiParams.hoodAngle;
    } else {
      // Fallback to traditional method
      targetRPM = ShootingConstants.getRPMForDistance(filteredDistanceToHub);
      targetAngle = ShootingConstants.getAngleForDistance(filteredDistanceToHub);
    }
    compensatedDistance = filteredDistanceToHub;

    // Update shooter subsystems
    drum.setVelocity(targetRPM);
    hood.setAngle(targetAngle);

    // --- SHOOTING SEQUENCE ---
    
    // Phase 1: Start feeding when aimed and at speed
    if (aimedAtHub && !feedingStarted && isDrumAtSpeed()) {
      feeder.setFeedVelocity(ShootingConstants.FEEDER_OUTPUT_RPM);
      feeder.setBoosterVelocity(ShootingConstants.FEEDER_OUTPUT_RPM * 0.5);
      conveyor.setVelocity(ShootingConstants.CONVEYOR_OUTPUT_RPM);
      
      // Only control intake if available - use slower speed during compression
      if (intake != null) {
        // Check if hopper is in compression state
        MasterStateMachine masterSM = MasterStateMachine.getInstance();
        boolean isCompressing = (masterSM != null &&
                              masterSM.getHopperState() == HopperState.COMPRESSING);
        
        if (isCompressing) {
          intake.setRollerVelocity(100.0); // Match compression state speed
        } else {
          intake.setRollerVelocity(ShootingConstants.INTAKE_ROLLER_OUTPUT_RPM);
        }
      }
      
      feedingStarted = true;
      feedingTimer.restart();
    }

    // Phase 2: Intake agitation 0.25s after feeding starts (only if intake available)
    if (feedingStarted && intake != null && feedingTimer.get() >= ShootingConstants.INTAKE_AGITATION_START) {
      double agitateElapsed = feedingTimer.get() - ShootingConstants.INTAKE_AGITATION_START;
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

    // Log shooting data
    logShootingData(
        headingError, desiredHeading, omegaRaw, omega, aimedAtHub,
        rawDistanceToHub, compensatedDistanceRaw, compensatedDistanceClamped,
        filteredDistanceToHub, radialVelocity, tangentialVelocity, tFlight,
        lateralLeadDistance, radialDistanceOffset, robotVelocityField, goalLocation);
  }

  /**
   * Calculates flight time based on distance and RPM.
   */
  private double calculateFlightTime(double distance) {
    if (distance >= ShootingConstants.MIN_SHOT_DISTANCE && distance <= ShootingConstants.MAX_SHOT_DISTANCE) {
      Double rpm = ShootingConstants.RPM_TABLE.get(distance);
      Double angle = ShootingConstants.ANGLE_TABLE.get(distance);
      if (rpm != null && angle != null) {
        double v_fw = rpm * 2 * Math.PI * ShootingConstants.FLYWHEEL_RADIUS_M / 60.0;
        double v_exit = ShootingConstants.FRICTION_COEFFICIENT * v_fw * ShootingConstants.COMPRESSION_FACTOR;
        double v_x = v_exit * Math.cos(Math.toRadians(angle));
        return distance / Math.max(v_x, 0.1);
      }
    }
    // Fallback
    return distance / 11.5;
  }

  /**
   * Checks if the drum is at the required shooting speed.
   */
  private boolean isDrumAtSpeed() {
    return Math.abs(drum.getAverageVelocityRPM() - targetRPM) < 50.0; // Within 50 RPM
  }

  /**
   * Logs shooting data for debugging and tuning.
   */
  private void logShootingData(
      double headingError, Rotation2d desiredHeading, double omegaRaw, double omega, boolean aimedAtHub,
      double rawDistance, double compensatedRaw, double compensatedClamped, double filtered,
      double radialVel, double tangentialVel, double flightTime, double lateralLead, double radialOffset,
      Translation2d robotVelocity, Translation2d goalLocation) {
    
    Logger.recordOutput("ShootingController/HeadingError_deg", Math.toDegrees(headingError));
    Logger.recordOutput("ShootingController/DesiredHeading_deg", desiredHeading.getDegrees());
    Logger.recordOutput("ShootingController/Omega_Raw_radps", omegaRaw);
    Logger.recordOutput("ShootingController/Omega_Filtered_radps", omega);
    Logger.recordOutput("ShootingController/AimedAtHub", aimedAtHub);
    
    Logger.recordOutput("ShootingController/Distance_Raw_m", compensatedRaw);
    Logger.recordOutput("ShootingController/Distance_Filtered_m", filtered);
    Logger.recordOutput("ShootingController/TargetRPM", targetRPM);
    Logger.recordOutput("ShootingController/TargetAngle_deg", targetAngle);
    
    Logger.recordOutput("ShootingController/RobotVelocity_X_mps", robotVelocity.getX());
    Logger.recordOutput("ShootingController/RobotVelocity_Y_mps", robotVelocity.getY());
    Logger.recordOutput("ShootingController/RadialVelocity_mps", radialVel);
    Logger.recordOutput("ShootingController/TangentialVelocity_mps", tangentialVel);
    Logger.recordOutput("ShootingController/FlightTime_s", flightTime);
    Logger.recordOutput("ShootingController/LateralLeadDistance_m", lateralLead);
    Logger.recordOutput("ShootingController/RadialDistanceOffset_m", radialOffset);
    Logger.recordOutput("ShootingController/CompensatedDistanceClamped_m", compensatedClamped);
    Logger.recordOutput("ShootingController/UncompensatedDistance_m", rawDistance);
    
    Translation2d virtualHub = goalLocation.minus(robotVelocity.times(flightTime));
    Logger.recordOutput("ShootingController/VirtualHub_X_m", virtualHub.getX());
    Logger.recordOutput("ShootingController/VirtualHub_Y_m", virtualHub.getY());
    
    Logger.recordOutput("ShootingController/FeedingStarted", feedingStarted);
    Logger.recordOutput("ShootingController/FeedingTimer_s", feedingTimer.get());
  }

  // Status getters

  public boolean isActive() {
    return active;
  }

  public boolean isAimed() {
    return active && Math.abs(MathUtil.angleModulus(
        angleController.getGoal().position - drive.getRotation().getRadians())) <= ShootingConstants.ANGLE_TOLERANCE_RADIANS;
  }

  public boolean isReadyToShoot() {
    return active && feedingStarted && isDrumAtSpeed();
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCompensatedDistance() {
    return compensatedDistance;
  }
}
