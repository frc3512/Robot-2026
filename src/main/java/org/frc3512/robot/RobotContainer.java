package org.frc3512.robot;

import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.commands.ShootAndMove;
import org.frc3512.robot.subsystems.climber.Climber;
import org.frc3512.robot.subsystems.climber.ClimberIO;
import org.frc3512.robot.subsystems.climber.ClimberIO_REAL;
import org.frc3512.robot.subsystems.climber.ClimberIO_SIM;
import org.frc3512.robot.subsystems.conveyor.Conveyor;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO_REAL;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO_SIM;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.drive.GyroIO;
import org.frc3512.robot.subsystems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsystems.drive.ModuleIO;
import org.frc3512.robot.subsystems.drive.ModuleIOSim;
import org.frc3512.robot.subsystems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsystems.drive.TunerConstants;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeConstants.IntakeState;
import org.frc3512.robot.subsystems.intake.IntakeIO;
import org.frc3512.robot.subsystems.intake.IntakeIO_REAL;
import org.frc3512.robot.subsystems.intake.IntakeIO_SIM;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO_REAL;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO_SIM;
import org.frc3512.robot.subsystems.shooter.flywheels.Flywheel;
import org.frc3512.robot.subsystems.shooter.flywheels.FlywheelIO;
import org.frc3512.robot.subsystems.shooter.flywheels.FlywheelIO_REAL;
import org.frc3512.robot.subsystems.shooter.flywheels.FlywheelIO_SIM;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO_REAL;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO_SIM;
import org.frc3512.robot.subsystems.vision.Vision;
import org.frc3512.robot.subsystems.vision.VisionConstants;
import org.frc3512.robot.subsystems.vision.VisionIO;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVision;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final Conveyor conveyor;
  private final Feeder feeder;
  private final Climber climber;

  // Timer for Hub Activity
  public Timer hubTimer = new Timer();

  public static String gameData;
  public static double prefire = 1; // Seconds before the hub becomes active to start shooting

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  // Enum for robot state
  enum RobotStates {
    STOWED,
    IDLING,
    INTAKING,
    PREPING_SHOT,
    SHOOTING,
    MANUAL_SHOOTONG,
    FERRYING
  }

  @AutoLogOutput(key = "Robot/Robot State")
  private RobotStates currentState = RobotStates.STOWED;

  // Track previous hub state for change detection
  // Initialized to true so no spurious notification fires on startup when
  // there is no FMS connection (isHubActive() returns true with no game data)
  private boolean wasHubActive = true;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate real hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.frontLeftCamera, VisionConstants.robotToLeft),
                new VisionIOPhotonVision(
                    VisionConstants.frontRightCamera, VisionConstants.robotToRight));

        flywheel = new Flywheel(new FlywheelIO_REAL());
        intake = new Intake(new IntakeIO_REAL());
        conveyor = new Conveyor(new ConveyorIO_REAL());
        hood = new Hood(new HoodIO_REAL());
        feeder = new Feeder(new FeederIO_REAL());
        climber = new Climber(new ClimberIO_REAL());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.frontLeftCamera, VisionConstants.robotToLeft, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.frontRightCamera,
                    VisionConstants.robotToRight,
                    drive::getPose));

        flywheel = new Flywheel(new FlywheelIO_SIM());
        intake = new Intake(new IntakeIO_SIM());
        conveyor = new Conveyor(new ConveyorIO_SIM());
        hood = new Hood(new HoodIO_SIM());
        feeder = new Feeder(new FeederIO_SIM());
        climber = new Climber(new ClimberIO_SIM());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        climber = new Climber(new ClimberIO() {});

        break;
    }

    registerNamedCommand("Hopper", intake.setPosition(IntakeState.EXTEND));
    registerNamedCommand("Intake", intake());
    registerNamedCommand("Shoot", autonShoot());
    registerNamedCommand("StopShoot", reset());
    registerNamedCommand("PrepShoot", idle());
    registerNamedCommand(
        "Climb", Commands.defer(() -> safeCommand(climber.climb()), java.util.Set.of()));
    registerNamedCommand("MidShoot", shootRaw(12, 3050));

    new EventTrigger("PrepIntake");

    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to build PathPlanner auto chooser. Check NamedCommands vs .auto files: "
              + e.getMessage(),
          e.getStackTrace());
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto (Error)", Commands.none());
    }

    // Configure the button bindings
    configureButtonBindings();

    // Set up auto routines
    SmartDashboard.putData("Auto Modes", autoChooser);
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // X the modules for a brake
    controller.leftStick().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when right stick is pressed
    // Pose est should handle this for us
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Secret command to make vision work
    controller.start().onTrue(new InstantCommand(() -> vision.work(true)));
    controller.back().onTrue(new InstantCommand(() -> vision.work(false)));

    // Left Side controls: Intake and idle
    controller.leftTrigger().onTrue(intake()).onFalse(prepShooting());
    controller.leftBumper().onTrue(idle());

    // Right side controls: Shoot and ferry
    controller.rightTrigger().whileTrue(autoShoot());
    controller.rightBumper().whileTrue(ferry()).onFalse(idle());

    // Climber Controls
    controller.y().onTrue(safeCommand(climber.raiseClimber()));
    controller.a().onTrue(safeCommand(climber.climb()));

    // Full reset in case something goes wrong
    controller.povLeft().onTrue(reset());

    // Dump Fuel
    controller.povUp().onTrue(dump()).onFalse(idle());
  }

  private void registerNamedCommand(String name, Command command) {
    NamedCommands.registerCommand(name, safeCommand(command));
  }

  private Command safeCommand(Command command) {
    if (command == null) {
      DriverStation.reportError(
          "Named command resolved to null. Using Commands.none() fallback.", false);
      return Commands.none();
    }
    return command;
  }

  // --- Begin Telop Commands ---

  // Methods
  public Command reset() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.STOWED),
        // Kill and retract Intake
        intake.setRollerSpeed(0),
        intake.setPosition(IntakeState.STOWED),
        // Stop Conveyor
        conveyor.setHopper(0.0),
        // Stop Feeder
        feeder.setFeeder(0.0),
        // Stop Flywheel
        flywheel.setRPM(0.0),
        // Bring Down Hood
        hood.setPosition(0),
        // Log action
        logMessage("Reseting Robot"),
        logMessage(
            Elastic.Notification.NotificationLevel.INFO,
            "Robot Reset",
            "Robot has been reset sucessfully",
            5000));
  }

  // Intake
  public Command intake() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.INTAKING),
        // Run intake rollers and extend
        intake.setPosition(IntakeState.EXTEND),
        intake.setRollerSpeed(0.90),
        // Log action
        logMessage("Begun Intaking"),
        // Wait
        Commands.waitSeconds(1),
        // Use hopper to push balls back
        conveyor.setHopper(0.2));
  }

  // Idling + Preping shot
  public Command idle() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.IDLING),
        // Stop Intake and bring it in
        intake.setRollerSpeed(0),
        intake.setPosition(IntakeState.EXTEND),
        // Stop Conveyor
        conveyor.setHopper(0.0),
        // Stop Feeder
        feeder.setFeeder(0.0),
        // Set Flywheel to idle values
        flywheel.setRPM(2000),
        hood.setPosition(5),
        // Log action
        logMessage("Idling"));
  }

  public Command prepShooting() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.PREPING_SHOT),
        // Keep Hooper open and intake slowed for indexing
        intake.setRollerSpeed(0.05),
        intake.setPosition(IntakeState.EXTEND),
        // Keep feeding off
        conveyor.setHopper(0.0),
        feeder.setFeeder(0.0),
        // Being speeding up flywheel
        flywheel.setRPM(2000),
        // Log action
        logMessage("Preping for shot"));
  }

  // Shoot
  public Command autoShoot() {
    return Commands.either(
        Commands.sequence(
            // Check if the hub is active before shooting, if not wait until it is
            Commands.waitUntil(this::isHubActive),
            // Update state
            Commands.runOnce(() -> currentState = RobotStates.SHOOTING),
            // Engage Shooting systems
            new ShootAndMove(
                drive,
                flywheel,
                hood,
                conveyor,
                feeder,
                intake,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX())),
        logMessage(
            Elastic.Notification.NotificationLevel.ERROR,
            "Cannot Shoot Now",
            "Hub is not active, do not shoot right now",
            5000),
        this::isHubActive);
  }

  public Command shoot() {
    return Commands.sequence(
        // Begin feeding balls
        conveyor.setHopper(0.7),
        feeder.setFeeder(0.9),
        // Log action
        logMessage("Shooting fuel"),
        // Wait for fuel to empty out to allow intake to agitate balls
        Commands.waitSeconds(0.25)
            // Bring intake in and out to agitate balls for 5 seconds
            .andThen(
                intake.setPosition(IntakeState.AGITATE),
                Commands.waitSeconds(0.25),
                intake.setPosition(IntakeState.EXTEND),
                Commands.waitSeconds(0.25))
            .repeatedly()
            .withTimeout(5),
        // Bring intake in
        intake.setPosition(IntakeState.AGITATE));
  }

  public Command autonShoot() {
    return Commands.sequence(
            // Check if the hub is active before shooting, if not wait until it is
            Commands.waitUntil(this::isHubActive),
            // Update state
            Commands.runOnce(() -> currentState = RobotStates.SHOOTING),
            // Engage Shooting systems
            new ShootAndMove(
                drive,
                flywheel,
                hood,
                conveyor,
                feeder,
                intake,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()))
        .withTimeout(5.0);
  }

  // Ferry from mid to our zone
  public Command ferry() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.FERRYING),
        // Point at drive station
        // DriveCommands.joystickDriveAtAngle(
        //     drive,
        //     () -> -controller.getLeftY(),
        //     () -> -controller.getLeftX(),
        //     () -> Rotation2d.k180deg),
        // Set hood to max
        hood.setPosition(22),
        // Spin flyhweels at a slower speed
        flywheel.setRPM(3300),
        // Allow spin-up time
        Commands.waitSeconds(1),
        // Feed into the shooter
        feeder.setFeeder(0.9),
        conveyor.setHopper(0.75),
        // Wait for a little bit of room
        Commands.waitSeconds(0.5),
        // Log action
        logMessage("Ferrying fuel to zone"),
        // Agitate intake repeatedly until command is cancelled
        Commands.sequence(
                intake.setPosition(IntakeState.EXTEND),
                Commands.waitSeconds(0.25),
                intake.setPosition(IntakeState.AGITATE),
                Commands.waitSeconds(0.25))
            .repeatedly());
  }

  public Command dump() {
    return Commands.sequence(
        // Setup intake
        intake.setPosition(IntakeState.EXTEND),
        intake.setRollerSpeed(-0.75),
        // Dump Conveyor
        conveyor.setHopper(-0.5),
        // Empty Feeder
        feeder.setFeeder(-0.5));
  }

  // --- Begin Auto Code ---

  // Auto chooser
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Auto Methods

  // ---- Begin Debuging and Tuning Commands ----

  // Feed raw degrees and rpms to get data for tree maps
  public Command shootRaw(double hoodDegrees, double flywheelRPM) {
    return Commands.sequence(
        // Set shooter to raw testing positions
        flywheel.setRPM(flywheelRPM),
        hood.setPosition(hoodDegrees),
        // Check boolean logic
        // Commands.waitUntil(hood.isAtTarget()),
        // Commands.waitUntil(flywheel.isVelocityWithinTolerance()),
        // Feed fuel
        conveyor.setHopper(0.7),
        feeder.setFeeder(0.9));
  }

  // Message logger
  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Robot/Command Log", message));
  }

  // Send message
  public Command logMessage(
      Elastic.Notification.NotificationLevel level,
      String title,
      String description,
      int displayTimeMillis) {
    return Commands.runOnce(
        () ->
            Elastic.sendNotification(
                new Elastic.Notification(level, title, description, displayTimeMillis)));
  }

  // Hub state change notifier — call this every periodic loop
  // Sends an Elastic notification whenever the hub transitions between active and inactive
  public void checkHubStateChange() {
    boolean hubActive = isHubActive();
    if (hubActive != wasHubActive) {
      if (hubActive) {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.Notification.NotificationLevel.INFO,
                "Hub Active",
                "The hub is now active. You may begin shooting.",
                5000));
      } else {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.Notification.NotificationLevel.WARNING,
                "Hub Inactive",
                "The hub is no longer active.",
                5000));
      }
      wasHubActive = hubActive;
    }
  }

  // --- Logic ---
  // public void runLedLogic() {
  //   switch (currentState) {
  //     case STOWED:
  //       leds.setPattern(leds.white);
  //       break;
  //     case IDLING:
  //       leds.setPattern(leds.red);
  //       break;
  //     case INTAKING:
  //       leds.setPattern(leds.orange);
  //       break;
  //     case PREPING_SHOT:
  //       leds.setPattern(leds.yellow);
  //       break;
  //     case SHOOTING:
  //       leds.setPattern(leds.green);
  //       break;
  //     case MANUAL_SHOOTONG:
  //       leds.setPattern(leds.green);
  //       break;
  //     case FERRYING:
  //       leds.setPattern(leds.purple);
  //       break;
  //   }
  // }

  // Hub activity logic,
  // returns a boolean that can be used to check if
  // the hub is active based on the game data and timer
  // Credit to 9084 for the logic
  @AutoLogOutput(key = "Robot/Hub Active")
  public boolean isHubActive() {
    double timer = hubTimer.get();
    gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return (timer <= 10
                || (timer >= (35 - prefire) && timer <= 60)
                || (timer >= (85 - prefire)));
          } else {
            return (timer <= 35)
                || (timer >= (60 - prefire) && timer <= 85)
                || (timer >= (110 - prefire));
          }
        case 'R':
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            return (timer <= 35)
                || (timer >= (60 - prefire) && timer <= 85)
                || (timer >= (110 - prefire));
          } else {
            return (timer <= 10
                || (timer >= (35 - prefire) && timer <= 60)
                || (timer >= (85 - prefire)));
          }

        default:
          return true;
      }
    } else {
      return true;
    }
  }
}
