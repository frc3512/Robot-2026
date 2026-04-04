package org.frc3512.robot;

import java.lang.annotation.ElementType;

import org.frc3512.robot.Elastic.Notification.NotificationLevel;
import org.frc3512.robot.commands.auto.AssistedAuto;
import org.frc3512.robot.commands.auto.SimpleCorrectedAuto;
import org.frc3512.robot.commands.auto.VisionGuidedAuto;
import org.frc3512.robot.commands.auto.PoseCorrector;
import org.frc3512.robot.commands.auto.VerifyPosition;
import org.frc3512.robot.commands.teleop.DriveCommands;
import org.frc3512.robot.commands.teleop.ShootAndMove;
import org.frc3512.robot.subsystems.Superstructure;
import org.frc3512.robot.subsystems.States;
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
import org.frc3512.robot.subsystems.vision.VisionCorrectionConstants;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final Conveyor conveyor;
  private final Feeder feeder;
  
  private final Superstructure superstructure;

  // Public accessors for vision correction commands
  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }

  // Timer for Hub Activity
  public Timer hubTimer = new Timer();

  public static String gameData;
  public static double prefire = 2; // Seconds before the hub becomes active to start shooting

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

  private char getWinner() {
    return Character.toUpperCase(gameData.trim().charAt(0));
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
        
        // Create superstructure with all subsystems and joystick suppliers
        superstructure = new Superstructure(
            drive,
            intake,
            conveyor,
            feeder,
            flywheel,
            hood,
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
        
        // Create superstructure with all subsystems and joystick suppliers
        superstructure = new Superstructure(
            drive,
            intake,
            conveyor,
            feeder,
            flywheel,
            hood,
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        
        // Create superstructure with all subsystems and joystick suppliers
        superstructure = new Superstructure(
            drive,
            intake,
            conveyor,
            feeder,
            flywheel,
            hood,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX());

        break;
    }

    //Named Commands
    registerNamedCommand("Hopper", intake.setPosition(IntakeState.EXTEND));
    registerNamedCommand("Intake", intake());
    registerNamedCommand("Shoot", autonShoot());
    registerNamedCommand("Reset", reset());
    registerNamedCommand("PrepShoot", idle());
    registerNamedCommand("StopIntake", stopIntake());

    //Event Triggers
    new EventTrigger("PrepIntake");
    new EventTrigger("KillIntake");
    new EventTrigger("EVerifyNZLeft");
    new EventTrigger("EVerifyNZRight");
    new EventTrigger("EVerifyHP");
    new EventTrigger("EVerifyShootMid");

    // Vision correction commands
    registerNamedCommand(
        "VisionCorrect", correctPoseWithVision(0.0, 0.0, 0.0)); // Generic corrector

    // Waypoint-specific verifiers using predefined poses
    registerNamedCommand("VerifyNZLeft", verifyPositionWithVision(
        VisionCorrectionConstants.WaypointPoses.NZ_LEFT_X,
        VisionCorrectionConstants.WaypointPoses.NZ_LEFT_Y,
        VisionCorrectionConstants.WaypointPoses.NZ_LEFT_HEADING_DEGREES));
    registerNamedCommand("VerifyNZRight", verifyPositionWithVision(
        VisionCorrectionConstants.WaypointPoses.NZ_RIGHT_X,
        VisionCorrectionConstants.WaypointPoses.NZ_RIGHT_Y,
        VisionCorrectionConstants.WaypointPoses.NZ_RIGHT_HEADING_DEGREES));
    registerNamedCommand("VerifyHP", verifyPositionWithVision(
        VisionCorrectionConstants.WaypointPoses.HP_X,
        VisionCorrectionConstants.WaypointPoses.HP_Y,
        VisionCorrectionConstants.WaypointPoses.HP_HEADING_DEGREES));
    registerNamedCommand("VerifyShootMid", verifyPositionWithVision(
        VisionCorrectionConstants.WaypointPoses.SHOOT_MID_X,
        VisionCorrectionConstants.WaypointPoses.SHOOT_MID_Y,
        VisionCorrectionConstants.WaypointPoses.SHOOT_MID_HEADING_DEGREES));

    // Set up auto routines without vision correction
    // createNormalAutos();
    // Set up auto routines with vision correction (commented out for now - needs testing)
    createVisionAutos();
    
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

    // Superstructure controls
    controller.leftTrigger().onTrue(
        Commands.runOnce(() -> superstructure.setWantedState(States.INTAKING))
    ).onFalse(
        Commands.runOnce(() -> superstructure.setWantedState(States.IDLE))
    );
    
    controller.leftBumper().onTrue(
        Commands.runOnce(() -> superstructure.setWantedState(States.IDLE))
    );
    
    controller.rightTrigger().whileTrue(
        Commands.runOnce(() -> superstructure.setWantedState(States.SHOOTING))
    ).onFalse(
        Commands.runOnce(() -> superstructure.setWantedState(States.IDLE))
    );
    
    controller.rightBumper().onTrue(
        Commands.runOnce(() -> superstructure.setWantedState(States.FERRYING))
    ).onFalse(
        Commands.runOnce(() -> superstructure.setWantedState(States.IDLE))
    );
    
    controller.povLeft().onTrue(
        Commands.runOnce(() -> superstructure.setWantedState(States.HOMED))
    );

    // X the modules for a brake
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
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

  public Command stopIntake(){
    return Commands.sequence(
      //Stop the intake
      intake.setRollerSpeed(0),
      intake.setPosition(IntakeState.EXTEND),
      //Log action
      logMessage("Stopping Intake"),
      logMessage(Elastic.Notification.NotificationLevel.INFO,
        "Intake Stopped",
        "Robot has stopped Intaking",
        5000));
  }

  // Intake
  public Command intake() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.INTAKING),
        // Run intake rollers and extend
        intake.setPosition(IntakeState.EXTEND),
        intake.setRollerSpeed(0.70),
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
        flywheel.setRPM(2800.0),
        hood.setPosition(5),
        // Log action
        logMessage("Idling"));
  }

  public Command prepShooting() {
    return Commands.sequence(
        // Update state
        Commands.runOnce(() -> currentState = RobotStates.PREPING_SHOT),
        intake.setRollerSpeed(0),
        intake.setPosition(IntakeState.EXTEND),
        // Keep feeding off
        conveyor.setHopper(0.0),
        feeder.setFeeder(0.0),
        // Being speeding up flywheel
        flywheel.setRPM(2800.0),
        // Log action
        logMessage("Preping for shot"));
  }

  // Shoot
  public Command autoShoot() {
    return Commands.sequence(
        // Wait until hub is active
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
            () -> -controller.getLeftX()));
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
        .withTimeout(3.5);
  }
  // --- Begin Auto Code ---

  // Auto chooser
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // --- Vision Correction Commands ---

  /** Creates a command that corrects robot pose using vision at a specific waypoint. */
  public Command correctPoseWithVision(
      double expectedX, double expectedY, double expectedHeadingDegrees) {
    return new PoseCorrector(
        drive,
        vision,
        new Pose2d(expectedX, expectedY, Rotation2d.fromDegrees(expectedHeadingDegrees)),
        VisionCorrectionConstants.VISION_CORRECTION_TOLERANCE_METERS,
        VisionCorrectionConstants.VISION_CORRECTION_ANGLE_TOLERANCE_DEGREES);
  }

  /** Creates a command that verifies robot position using vision before proceeding. */
  public Command verifyPositionWithVision(
      double expectedX, double expectedY, double expectedHeadingDegrees) {
    Pose2d expectedPose = new Pose2d(expectedX, expectedY, Rotation2d.fromDegrees(expectedHeadingDegrees));
    return new VerifyPosition(
        vision,
        expectedPose,
        VisionCorrectionConstants.VISION_CORRECTION_TOLERANCE_METERS,
        VisionCorrectionConstants.VISION_CORRECTION_ANGLE_TOLERANCE_DEGREES,
        2.0); // 2 second timeout
  }

  /** Wraps a PathPlanner command with vision assistance for continuous pose correction. */
  public Command withVisionAssistance(Command pathCommand) {
    return new VisionGuidedAuto(drive, vision, pathCommand);
  }

  /**
   * Example method showing how to create a vision-corrected autonomous command.
   * Uses the simplified vision guidance system.
   */
  public Command createVisionCorrectedAuto(String autoName) {
    try {
      // Get the original PathPlanner auto command
      Command originalAuto = AutoBuilder.buildAuto(autoName);

      // Wrap it with simplified vision guidance
      return SimpleCorrectedAuto.withVisionGuidance(originalAuto, this);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to create vision-corrected auto: " + autoName, e.getStackTrace());
      return Commands.none();
    }
  }

  /** Sets up the auto chooser with vision-corrected versions of all available autos. */
  private void createVisionAutos() {
    autoChooser = new SendableChooser<>();
    
    // List of all available auto names (without .auto extension)
    String[] autoNames = {
      "NzDoubleLeft",
      "NzDoubleRight",
      "NzLeft",
      "NzRight",
      "NzTrenchDoubleLeft",
      "NzTrenchDoubleRight",
      "NzTrenchLeft",
      "NzTrenchRight",
      "zNzThenDepot",
      "zNzThenHp",
      "NzTrenchDepot",
      "NzTrenchHp",
      "HpScoreAuto",
      "HpToNzAuto",
      "depotAuto",
      "midHpAuto",
      "shootAuto"
    };
    
    boolean hasDefault = false;
    
    // Add each auto with vision correction to the chooser
    for (String autoName : autoNames) {
      try {
        Command visionCorrectedAuto = createVisionCorrectedAuto(autoName);
        if (!hasDefault) {
          autoChooser.setDefaultOption(autoName, visionCorrectedAuto);
          hasDefault = true;
        } else {
          autoChooser.addOption(autoName, visionCorrectedAuto);
        }
      } catch (Exception e) {
        DriverStation.reportWarning(
            "Failed to load auto '" + autoName + "': " + e.getMessage(), 
            e.getStackTrace());
      }
    }
    
    // Add fallback option in case all autos fail to load
    if (!hasDefault) {
      autoChooser.setDefaultOption("No Auto (Error)", Commands.none());
    }
  }

  public void createNormalAutos() {
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
  // Credit to 9084 for the idea (they are so cool btw)
  @AutoLogOutput(key = "Robot/Hub Active")
  public boolean isHubActive() {
    // Updated for 130s teleop: first 10s both active, then 100s alternating 25s periods,
    // last 30s both active. Alternating relative to elapsed 10s start.
    double elapsed = hubTimer.get();

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
    Logger.recordOutput("Robot/Hub Debug IsPrefire", 
      (elapsed >= 8.0 && elapsed < 10.0) || 
      (isMatch && ((phase == 0 && elapsed >= 23.0 && elapsed < 25.0) ||
                 (phase == 2 && elapsed >= 73.0 && elapsed < 75.0)) ||
      (!isMatch && ((phase == 3 && elapsed >= 48.0 && elapsed < 50.0) ||
                  (phase == 1 && elapsed >= 23.0 && elapsed < 25.0)))));

    return ourSideActive;
  }
}
