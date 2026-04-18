package org.frc3512.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import org.frc3512.robot.Constants;
import org.frc3512.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class Aim extends Command {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 18;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 10.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  // --- Jitter compensation ---
  // Suppress micro-corrections when the robot is already within 2° of the target heading.
  // Pose-estimator noise can produce sub-degree phantom errors that cause constant small
  // oscillations; this deadband prevents the PID from reacting to them.
  private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2);

  // --- Tunable angle offset for clockwise rotation ---
  // Adjust this value to fine-tune the robot's shooting angle offset.
  // Positive values rotate the target heading counter-clockwise (in degrees).
  private static final double ANGLE_OFFSET_DEGREES = 0.0;

  // Physics parameters for dynamic ball velocity calculation
  private static final double FLYWHEEL_RADIUS_M = 0.0508; // 4" diameter flywheel
  private static final double FRICTION_COEFFICIENT = 0.55; // μ rubber/foam
  private static final double COMPRESSION_FACTOR = 1.08; // Grip enhancement

  // Gain multipliers for compensation tuning.
  private static final double LATERAL_COMPENSATION_GAIN = 3.75;
  private static final double RADIAL_COMPENSATION_GAIN = -0.5;

  // Clamp effective distance to stay within interpolation table range
  // and avoid null setpoints from map lookups.
  private static final double MIN_SHOT_DISTANCE = 1.25;
  private static final double MAX_SHOT_DISTANCE = 6.33;

  // Single-pole IIR time constants (seconds).  Chosen so the filters are fast enough
  // to track real motion but slow enough to reject high-frequency pose-noise spikes.
  //   omega    TC=0.06 s  →  cutoff ≈ 2.65 Hz  (smooths angular output)
  //   distance TC=0.10 s  →  cutoff ≈ 1.59 Hz  (smooths RPM / hood-angle setpoints)
  private static final double OMEGA_FILTER_TC = 0.06;
  private static final double DISTANCE_FILTER_TC = 0.10;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final ProfiledPIDController angleController;

  // Low-pass filters — instantiated once, reset on initialize() to avoid stale state.
  private final LinearFilter omegaFilter = LinearFilter.singlePoleIIR(OMEGA_FILTER_TC, 0.02);
  private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(DISTANCE_FILTER_TC, 0.02);

  private static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();

  // Store current distance for @AutoLogOutput
  private double currentDistanceToHub = 0.0;
  private boolean aimedAtTarget = false;
  private double wantedRPM = 0.0;
  private double wantedAngle = 0.0;

  @AutoLogOutput(key = "Robot/Distance From Hub")
  public double getDistanceToHub() {
    return currentDistanceToHub;
  }

  public Aim(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // ( DISTANCE (m) || RPM )
    RPM_TABLE.put(2.06, 1800.0);
    RPM_TABLE.put(2.86, 2000.0);
    RPM_TABLE.put(3.56, 2150.0);
    RPM_TABLE.put(3.95, 2100.0);
    RPM_TABLE.put(4.44, 2300.0);
    RPM_TABLE.put(5.36, 2400.0);

    // ( DISTANCE (m) || HOOD ANGLE (degrees) )
    ANGLE_TABLE.put(2.06, 10.0);
    ANGLE_TABLE.put(2.86, 13.0);
    ANGLE_TABLE.put(3.56, 21.0);
    ANGLE_TABLE.put(3.95, 19.0);
    ANGLE_TABLE.put(4.44, 24.0);
    ANGLE_TABLE.put(5.36, 29.0);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
    // Clear filter history so a previous run's state doesn't cause a startup transient.
    omegaFilter.reset();
    distanceFilter.reset();
  }

  @Override
  public void execute() {

    // --- AIM SEQUENCE ---

    // Get linear velocity from joysticks
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();

    // Get current pose and speeds
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();

    // Convert robot-relative chassis speeds to a field-relative velocity vector.
    // getChassisSpeeds() returns speeds in the robot frame; rotating by the robot's
    // current heading transforms them into the field (world) frame.
    Translation2d robotVelocityField =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .rotateBy(drive.getRotation());

    // 1. LATENCY COMP
    double latency = 0.15; // Tuned constant
    Translation2d futurePos = robotPose.getTranslation().plus(robotVelocityField.times(latency));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = Constants.FieldConstants.hubPosition();
    Translation2d targetVec = goalLocation.minus(futurePos);

    // 3. MOVING-WHILE-SHOOTING COMPENSATION
    // Decompose velocity into radial (toward/away hub) and tangential (left/right around hub)
    // components, then compensate each axis with separate gains:
    //  • Tangential -> heading lead (robot angle compensation)
    //  • Radial     -> effective shot distance (hood/flywheel compensation)
    double rawDistanceToHub = targetVec.getNorm();

    // Dynamic ball horizontal speed from RPM table (distance-aware)
    double tFlight;
    if (rawDistanceToHub >= MIN_SHOT_DISTANCE && rawDistanceToHub <= MAX_SHOT_DISTANCE) {
      Double rpm = RPM_TABLE.get(rawDistanceToHub);
      Double theta_deg = ANGLE_TABLE.get(rawDistanceToHub);
      if (rpm != null && theta_deg != null) {
        double v_fw = rpm * 2 * Math.PI * FLYWHEEL_RADIUS_M / 60.0;
        double v_exit = FRICTION_COEFFICIENT * v_fw * COMPRESSION_FACTOR;
        double v_x = v_exit * Math.cos(Math.toRadians(theta_deg));
        tFlight = rawDistanceToHub / Math.max(v_x, 0.1); // Avoid div/0
      } else {
        // Fallback
        tFlight = rawDistanceToHub / 11.5;
      }
    } else {
      // Fallback for out-of-range distances
      tFlight = rawDistanceToHub / 11.5;
    }

    // Unit vectors in radial and tangential directions.
    Translation2d radialUnit =
        rawDistanceToHub > 1e-6 ? targetVec.div(rawDistanceToHub) : Translation2d.kZero;
    Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

    // Signed velocity components in m/s.
    double radialVelocity =
        robotVelocityField.getX() * radialUnit.getX()
            + robotVelocityField.getY() * radialUnit.getY();
    double tangentialVelocity =
        robotVelocityField.getX() * tangentialUnit.getX()
            + robotVelocityField.getY() * tangentialUnit.getY();

    // Angle lead from tangential component.
    // Sign inverted so the robot leads opposite the perceived miss direction.
    double lateralLeadDistance = -tangentialVelocity * tFlight * LATERAL_COMPENSATION_GAIN;
    double baseHeadingRadians = Math.atan2(targetVec.getY(), targetVec.getX());
    double compensatedHeadingRadians =
        baseHeadingRadians + Math.atan2(lateralLeadDistance, rawDistanceToHub);
    // Apply tunable clockwise angle offset
    compensatedHeadingRadians += Math.toRadians(ANGLE_OFFSET_DEGREES);
    Rotation2d desiredHeading = new Rotation2d(compensatedHeadingRadians + Math.PI);
    
    // Log angle offset for tuning purposes
    Logger.recordOutput("ShootAndMove/AngleOffsetDegrees", ANGLE_OFFSET_DEGREES);
    Logger.recordOutput("ShootAndMove/BaseHeadingDegrees", Math.toDegrees(baseHeadingRadians));
    Logger.recordOutput("ShootAndMove/CompensatedHeadingDegrees", Math.toDegrees(compensatedHeadingRadians));

    // Radial compensation adjusts effective distance for hood/flywheel.
    // Sign inverted so approach/retreat compensation goes opposite previous behavior.
    double radialDistanceOffset = radialVelocity * tFlight * RADIAL_COMPENSATION_GAIN;
    double compensatedDistanceRaw = rawDistanceToHub + radialDistanceOffset;
    double compensatedDistanceClamped =
        MathUtil.clamp(compensatedDistanceRaw, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);
    
    // Update distance for @AutoLogOutput
    currentDistanceToHub = rawDistanceToHub;

    // Keep a virtual hub for diagnostics only (combined velocity model).
    Translation2d virtualHub = goalLocation.minus(robotVelocityField.times(tFlight));

    // Calculate angular error (normalised to [-π, π]).
    // Apply a deadband: if the robot is already within ANGLE_TOLERANCE_RADIANS of the
    // target, output zero so pose-estimator noise doesn't drive constant micro-corrections.
    double headingError =
        MathUtil.angleModulus(desiredHeading.getRadians() - drive.getRotation().getRadians());
    double omegaRaw = 0.0;
    if (Math.abs(headingError) > ANGLE_TOLERANCE_RADIANS) {
      omegaRaw =
          angleController.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());
    }
    // Low-pass filter the omega output to smooth residual jitter from pose-estimator noise.
    double omega = omegaFilter.calculate(omegaRaw);

    // --- AdvantageScope tuning logs ---
    boolean aimedAtHub = Math.abs(headingError) <= ANGLE_TOLERANCE_RADIANS;
    aimedAtTarget = aimedAtHub;
    Logger.recordOutput("ShootAndMove/HeadingError_deg", Math.toDegrees(headingError));
    Logger.recordOutput("ShootAndMove/DesiredHeading_deg", desiredHeading.getDegrees());
    Logger.recordOutput("ShootAndMove/Omega_Raw_radps", omegaRaw);
    Logger.recordOutput("ShootAndMove/Omega_Filtered_radps", omega);
    Logger.recordOutput("Robot/AimedAtHub", aimedAtHub);

    // SET OUTPUTS
    // Point robot at hub (set drive heading)
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    // 4. SET FLYWHEEL RPM AND HOOD ANGLE BASED ON RADIAL-COMPENSATED DISTANCE
    // Low-pass filter to reduce setpoint noise and clamp to valid interpolation range.
    double filteredDistanceToHub = distanceFilter.calculate(compensatedDistanceClamped);

    Double rpm = RPM_TABLE.get(filteredDistanceToHub);
    Double angle = ANGLE_TABLE.get(filteredDistanceToHub);

    wantedRPM = rpm;
    wantedAngle = angle;

    // --- AdvantageScope tuning logs ---
    Logger.recordOutput("ShootAndMove/Distance_Raw_m", compensatedDistanceRaw);
    Logger.recordOutput("ShootAndMove/Distance_Filtered_m", filteredDistanceToHub);
    Logger.recordOutput("ShootAndMove/Flywheel_RPM_Setpoint", rpm);
    Logger.recordOutput("ShootAndMove/Hood_Angle_Setpoint_deg", angle);
    // Moving-while-shooting compensation diagnostics
    Logger.recordOutput("ShootAndMove/RobotVelocity_X_mps", robotVelocityField.getX());
    Logger.recordOutput("ShootAndMove/RobotVelocity_Y_mps", robotVelocityField.getY());
    Logger.recordOutput("ShootAndMove/RadialVelocity_mps", radialVelocity);
    Logger.recordOutput("ShootAndMove/TangentialVelocity_mps", tangentialVelocity);
    Logger.recordOutput("ShootAndMove/FlightTime_s", tFlight);
    Logger.recordOutput("ShootAndMove/LateralLeadDistance_m", lateralLeadDistance);
    Logger.recordOutput("ShootAndMove/RadialDistanceOffset_m", radialDistanceOffset);
    Logger.recordOutput("ShootAndMove/CompensatedDistanceClamped_m", compensatedDistanceClamped);
    Logger.recordOutput("ShootAndMove/VirtualHub_X_m", virtualHub.getX());
    Logger.recordOutput("ShootAndMove/VirtualHub_Y_m", virtualHub.getY());
    Logger.recordOutput("ShootAndMove/UncompensatedDistance_m", rawDistanceToHub);
  }

  public boolean isAimed() {
    return aimedAtTarget;
  }

  public double getAngle() {
    return wantedAngle;
  }

  public double getRPM() {
    return wantedRPM;
  }

  @Override
  public void end(boolean interrupted) {}

  private Translation2d getLinearVelocityFromJoysticks() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}
