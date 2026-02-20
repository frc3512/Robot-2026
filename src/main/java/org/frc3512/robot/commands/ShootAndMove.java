package org.frc3512.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import java.util.function.DoubleSupplier;
import org.frc3512.robot.Constants;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.shooter.flywheels.Flywheel;
import org.frc3512.robot.subsystems.shooter.hood.Hood;

public class ShootAndMove extends Command {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 12;
  private static final double ANGLE_KD = 0;
  private static final double ANGLE_MAX_VELOCITY = 10.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final ProfiledPIDController angleController;

  private static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();

  public ShootAndMove(
      Drive drive, Flywheel flywheel, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
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
    RPM_TABLE.put(2.0, 2600.0);
    RPM_TABLE.put(4.0, 3750.0);
    RPM_TABLE.put(5.0, 4100.0);

    // ( DISTANCE (m) || HOOD ANGLE (degrees) )
    ANGLE_TABLE.put(0.0, 0.0);

    addRequirements(drive, flywheel);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    // Get linear velocity from joysticks
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();

    // Get current pose and speeds
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeed = drive.getChassisSpeeds();

    // 1. LATENCY COMP
    double latency = 0.15; // Tuned constant
    Translation2d futurePos =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
                    .times(latency));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = Constants.FieldConstants.GOAL_LOCATION;
    Translation2d targetVec = goalLocation.minus(futurePos);

    // Desired heading is the angle to the goal
    Rotation2d desiredHeading = targetVec.getAngle();

    // Calculate angular speed to turn to desired heading
    double omega =
        angleController.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());

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

    // 3. SET FLYWHEEL RPM AND HOOD ANGLE BASED ON DISTANCE
    double distanceToHub = targetVec.getNorm();

    Double rpm = RPM_TABLE.get(distanceToHub);
    Double angle = ANGLE_TABLE.get(distanceToHub);

    flywheel.setRPMDirect(rpm);
    hood.setPositionDirect(angle);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // Stop flywheel and set hood to lowered position for safety
    flywheel.stop();
    hood.setPosition(2);
  }

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
