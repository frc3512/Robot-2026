package org.frc3512.robot;

import org.frc3512.robot.commands.teleop.DriveCommands;
import org.frc3512.robot.commands.teleop.Aim;
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
import org.frc3512.robot.subsystems.intake.IntakeIO;
import org.frc3512.robot.subsystems.intake.IntakeIO_REAL;
import org.frc3512.robot.subsystems.intake.IntakeIO_SIM;
import org.frc3512.robot.subsystems.vision.Vision;
import org.frc3512.robot.subsystems.vision.VisionConstants;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVision;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.frc3512.robot.subsystems.shooter.drum.Flywheel;
import org.frc3512.robot.subsystems.shooter.drum.FlywheelIO;
import org.frc3512.robot.subsystems.shooter.drum.FlywheelIO_REAL;
import org.frc3512.robot.subsystems.shooter.drum.FlywheelIO_SIM;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO_REAL;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO_SIM;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO_REAL;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO_SIM;
import org.frc3512.robot.util.StateManager;
import org.frc3512.robot.util.StateManager.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final Conveyor conveyor;
  private final Feeder feeder;
  private final Vision vision;

  private final StateManager stateManager;

  // Game data and timing
  public static String gameData;
  public static double prefire = 2; // Seconds before the hub becomes active to start shooting

  private char getWinner() {
    return Character.toUpperCase(gameData.trim().charAt(0));
  }

  // Public accessors for vision correction commands
  public Drive getDrive() {
    return drive;
  }


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  // Track previous hub state for change detection
  // Initialized to true so no spurious notification fires on startup when
  // there is no FMS connection (isHubActive() returns true with no game data)
  private boolean wasHubActive = true;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    registerEventTrigger();

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

        flywheel = new Flywheel(new FlywheelIO_REAL());
        intake = new Intake(new IntakeIO_REAL());
        conveyor = new Conveyor(new ConveyorIO_REAL());
        hood = new Hood(new HoodIO_REAL());
        feeder = new Feeder(new FeederIO_REAL());

        // Initialize vision with PhotonVision cameras
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.frontLeftCamera, VisionConstants.robotToLeft),
            new VisionIOPhotonVision(VisionConstants.frontRightCamera, VisionConstants.robotToRight));

        stateManager =
            new StateManager(
                conveyor,
                intake,
                flywheel,
                feeder,
                hood,
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());

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

        flywheel = new Flywheel(new FlywheelIO_SIM());
        intake = new Intake(new IntakeIO_SIM());
        conveyor = new Conveyor(new ConveyorIO_SIM());
        hood = new Hood(new HoodIO_SIM());
        feeder = new Feeder(new FeederIO_SIM());

        // Initialize vision with simulated PhotonVision cameras
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(VisionConstants.frontLeftCamera, VisionConstants.robotToLeft, drive::getPose),
            new VisionIOPhotonVisionSim(VisionConstants.frontRightCamera, VisionConstants.robotToRight, drive::getPose));

        stateManager =
            new StateManager(
                conveyor,
                intake,
                flywheel,
                feeder,
                hood,
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());

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

        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});

        // Initialize vision with no IO for replay mode
        vision = new Vision(drive::addVisionMeasurement);

        stateManager =
            new StateManager(
                conveyor,
                intake,
                flywheel,
                feeder,
                hood,
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());

        break;
    }

    // Initialize BLine global constraints
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        4.5, // maxVelocityMetersPerSec
        10.0, // maxAccelerationMetersPerSec2
        600, // maxVelocityDegPerSec
        2000, // maxAccelerationDegPerSec2
        0.03, // endTranslationToleranceMeters
        2.0, // endRotationToleranceDeg
        0.25 // intermediateHandoffRadiusMeters
    ));

    // Set up auto routines
    createNormalAutos();

    // Configure the button bindings
    configureButtonBindings();

    // Set up auto routines
    SmartDashboard.putData("Auto Modes", autoChooser);
  }

  //This is where EventTriggers(NamedCommands) are registered
  //Code format taken from frc 2638 Rebel Robotics
  private void registerEventTrigger() {

    FollowPath.registerEventTrigger("Shoot", () -> stateManager.setWantedState(RobotState.SHOOTING));

    FollowPath.registerEventTrigger("Intake", () -> stateManager.setWantedState(RobotState.INTAKING));

    FollowPath.registerEventTrigger("Reset", () -> stateManager.setWantedState(RobotState.HOME));
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

    // Superstructure controls
    controller.leftTrigger().onTrue(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.INTAKING))
    ).onFalse(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.IDLE))
    );
    controller.leftBumper().onTrue(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.FERRY))
    ).onFalse(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.IDLE))
    );
    
    controller.rightTrigger().whileTrue(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.SHOOTING))
    ).onFalse(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.IDLE))
    );
    controller.rightBumper().whileTrue(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.DUMPING))
    ).onFalse(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.IDLE))
    );

    controller.povUp().onTrue(
      Commands.runOnce(() -> stateManager.setWantedState(RobotState.HOME))
    );

  }
  
  // --- Begin Auto Code ---

  // Auto chooser
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Creates a BLine path command from a path name. */
  public Command createBLinePath(String pathName) {
    try {
      Path path = new Path(pathName);
      return drive.getPathBuilder().build(path);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load BLine path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
  }

  /** Gets the first path name from the auto chooser for pre-match orientation. */
  public String getFirstPathName() {
    try {
      Command selectedAuto = autoChooser.getSelected();
      if (selectedAuto != null) {
        // Extract path name from the command name/chooser selection
        // This is a simplified approach - in practice you might need to 
        // store the path name alongside the command in the chooser
        String commandName = selectedAuto.getName();
        if (commandName != null && !commandName.isEmpty()) {
          return commandName;
        }
      }
    } catch (Exception e) {
      // Silently fail
    }
    return null;
  }


  /** Sets up the auto chooser with BLine path commands. */
  private void createNormalAutos() {
    autoChooser = new SendableChooser<>();
    
    // Set up individual auto commands
    autoChooser.setDefaultOption("None", Commands.none());
  
  }

  // Excess Logic

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
    try {
      if (hubActive != wasHubActive) {
        if (hubActive) {
          Elastic.sendNotification(
              new Elastic.Notification(
                  Elastic.Notification.NotificationLevel.INFO,
                  "Hub Active",
                  "The hub is now active. You may begin shooting.",
                  5000,
                  500,
                  -2));
        } else {
          Elastic.sendNotification(
              new Elastic.Notification(
                  Elastic.Notification.NotificationLevel.WARNING,
                  "Hub Inactive",
                  "The hub is no longer active.",
                  5000,
                  500,
                  -2));
        }
        wasHubActive = hubActive;
      }
    } catch (Exception e) {
      return;
    }
  }

      // --- Logic methods
    // Hub activity logic,
    // returns a boolean that can be used to check if
    // the hub is active based on the game data and timer
    // Credit to 9084 for the idea (they are so cool btw)
    @AutoLogOutput(key = "Robot/Hub Active")
    public boolean isHubActive() {
    // Updated for 130s teleop: first 10s both active, then 100s alternating 25s periods,
    // last 30s both active. Alternating relative to elapsed 10s start.
    double elapsed = Constants.GeneralConstants.hubTimer.get();

    gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isBlank()) {
      return true;
    }

    if (elapsed < 10.0 || elapsed >= 110.0) {
      // First 10s or last 30s (110-130+): both hubs active -> our hub active
      return true;
    }

    // Prefire period: 2 seconds before hub becomes active
    if (elapsed >= 8.0 && elapsed < 10.0) {
      // 2-second prefire before the 10s both-active period
      return true;
    }

    // Alternating phase: 10-110 elapsed (100s total), 25s periods
    double phaseTime = elapsed - 10.0;
    int phase = (int) Math.floor(phaseTime / 25.0) % 4;

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    char winner = getWinner();

    // Determine if our side is active in this phase
    // Match (our win): active phases 1 (25-50), 3 (75-100)
    // Mismatch (opp win): active phases 0 (0-25), 2 (50-75)
    boolean ourSideActive;

    boolean isMatch =
        (winner == 'B' && alliance == Alliance.Blue) || (winner == 'R' && alliance == Alliance.Red);

    // Check for prefire periods (2 seconds before each active phase)
    if (isMatch) {
      // Match: Prefire before phases 1 (23-25) and 3 (73-75)
      if ((phase == 0 && elapsed >= 23.0 && elapsed < 25.0) ||
          (phase == 2 && elapsed >= 73.0 && elapsed < 75.0)) {
        return true;
      }
      ourSideActive = (phase == 1 || phase == 3);
    } else {
      // Mismatch: Prefire before phases 0 (0-2) and 2 (48-50)
      if ((phase == 3 && elapsed >= 48.0 && elapsed < 50.0) ||
          (phase == 1 && elapsed >= 23.0 && elapsed < 25.0)) {
        return true;
      }
      ourSideActive = (phase == 0 || phase == 2);
    }

    Logger.recordOutput("Robot/Hub Debug Elapsed", elapsed);
    Logger.recordOutput("Robot/Hub Debug Phase", phase);
    Logger.recordOutput("Robot/Hub Debug Winner", String.valueOf(winner));
    Logger.recordOutput("Robot/Hub Debug Alliance", alliance.toString());
    Logger.recordOutput("Robot/Hub Debug OurSideActive", ourSideActive);
    Logger.recordOutput("Robot/Hub Debug IsMatch", isMatch);

    return ourSideActive;
  }

}
