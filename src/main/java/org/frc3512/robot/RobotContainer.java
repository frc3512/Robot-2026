package org.frc3512.robot;

import org.frc3512.robot.ai.EnhancedShootingController;
import org.frc3512.robot.commands.StateCommands;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.drive.GyroIO;
import org.frc3512.robot.subsystems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsystems.drive.ModuleIO;
import org.frc3512.robot.subsystems.drive.ModuleIOSim;
import org.frc3512.robot.subsystems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsystems.drive.TunerConstants;
import org.frc3512.robot.subsystems.hopper.conveyor.Conveyor;
import org.frc3512.robot.subsystems.hopper.conveyor.ConveyorIO;
import org.frc3512.robot.subsystems.hopper.conveyor.ConveyorIOSim;
import org.frc3512.robot.subsystems.hopper.conveyor.ConveyorIOTalonFX;
import org.frc3512.robot.subsystems.hopper.intake.Intake;
import org.frc3512.robot.subsystems.hopper.intake.IntakeIO;
import org.frc3512.robot.subsystems.hopper.intake.IntakeIOSim;
import org.frc3512.robot.subsystems.hopper.intake.IntakeIOTalonFX;
import org.frc3512.robot.subsystems.shooter.drum.Drum;
import org.frc3512.robot.subsystems.shooter.drum.DrumIO;
import org.frc3512.robot.subsystems.shooter.drum.DrumIOSim;
import org.frc3512.robot.subsystems.shooter.drum.DrumIOTalonFX;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIO;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIOSim;
import org.frc3512.robot.subsystems.shooter.feeder.FeederIOTalonFX;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.shooter.hood.HoodIO;
import org.frc3512.robot.subsystems.shooter.hood.HoodIOSim;
import org.frc3512.robot.subsystems.shooter.hood.HoodIOTalonFX;
import org.frc3512.robot.subsystems.statemachine.MasterStateMachine;
import org.frc3512.robot.subsystems.vision.Vision;
import org.frc3512.robot.subsystems.vision.VisionConstants;
import org.frc3512.robot.subsystems.states.RobotState;
import org.frc3512.robot.subsystems.vision.VisionIO;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVision;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.frc3512.robot.subsystems.leds.LED;
import org.frc3512.robot.subsystems.leds.LEDIO;
import org.frc3512.robot.subsystems.leds.LEDIOAddressableLED;
import org.frc3512.robot.subsystems.leds.LEDIOSim;
import org.frc3512.robot.subsystems.leds.LEDPatterns;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final Intake intake;
  private final Conveyor conveyor;
  private final Feeder feeder;
  private final Hood hood;
  private final Drum drum;
  private final LED leds;
  private final EnhancedShootingController enhancedShootingController;
  
  // State machine and state commands
  private final MasterStateMachine stateMachine;
  private final StateCommands stateCommands;

  // Game data and timing
  public static String gameData;
  public static double prefire = 2; // Seconds before the hub becomes active to start shooting

  private char getWinner() {
    return Character.toUpperCase(gameData.trim().charAt(0));
  }

  private boolean wasHubActive = true;

  
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  
  // Dashboard
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and state machine. */
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

        // New subsystems with real hardware
        intake = new Intake(new IntakeIOTalonFX(21, 22, 20));
        conveyor = new Conveyor(new ConveyorIOTalonFX(23));
        feeder = new Feeder(new FeederIOTalonFX(24, 25));
        hood = new Hood(new HoodIOTalonFX(26));
        drum = new Drum(new DrumIOTalonFX(30, 31, 32));
        leds = new LED(new LEDIOAddressableLED(0, LEDPatterns.LED_COUNT));
        
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

        // New subsystems with simulation
        intake = new Intake(new IntakeIOSim());
        conveyor = new Conveyor(new ConveyorIOSim());
        feeder = new Feeder(new FeederIOSim());
        hood = new Hood(new HoodIOSim());
        drum = new Drum(new DrumIOSim());
        leds = new LED(new LEDIOSim(LEDPatterns.LED_COUNT));
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
        
        // New subsystems with disabled IO
        intake = new Intake(new IntakeIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        feeder = new Feeder(new FeederIO() {});
        hood = new Hood(new HoodIO() {});
        drum = new Drum(new DrumIO() {});
        leds = new LED(new LEDIO() {});
        break;
    }

    // Initialize enhanced shooting controller
    enhancedShootingController = new EnhancedShootingController(drive, drum, hood, feeder);
    
    // Create state machine and state commands
    stateMachine = new MasterStateMachine(drive, drum, feeder, hood, intake, conveyor, leds, vision, this);
    
    // Set enhanced shooting controller in state machine
    stateMachine.setEnhancedShootingController(enhancedShootingController);
    stateCommands = new StateCommands(stateMachine);
    
    // Initialize LEDs to idle state
    leds.setSolidColor(LEDPatterns.BLUE[0], LEDPatterns.BLUE[1], LEDPatterns.BLUE[2]);
    
    // Set up auto routines
    SmartDashboard.putData("Auto Modes", autoChooser);
  }
  
  /** Called periodically to check button states and update state machine */
  public void checkButtonStates() {
    // Priority-based button handling - higher priority buttons checked first
    RobotState activeState = determineActiveButtonState();
    
    // Apply the determined state
    switch (activeState) {
      case HOME:
        stateCommands.toHome();
        break;
      case DUMPING:
        stateCommands.toDumping();
        break;
      case AIMING:
        stateCommands.toAiming();
        break;
      case INTAKING:
        stateCommands.toIntaking();
        break;
      case FERRYING:
        stateCommands.toFerrying();
        break;
      case IDLE:
      default:
        stateCommands.toIdle();
        break;
    }
    
    // Handle shooting button release separately
    if (!controller.rightTrigger().getAsBoolean()) {
      stateCommands.stopShooting();
    }
  }
  
  /**
   * Determines the highest priority active button state.
   * Priority order: HOME > DUMPING > AIMING > INTAKING > FERRYING > IDLE
   */
  private RobotState determineActiveButtonState() {
    // Start button has highest priority (HOME)
    if (controller.start().getAsBoolean()) {
      return RobotState.HOME;
    }
    
    // Left bumper (DUMPING) has second priority
    if (controller.leftBumper().getAsBoolean()) {
      return RobotState.DUMPING;
    }
    
    // Right trigger (AIMING) has third priority
    if (controller.rightTrigger().getAsBoolean()) {
      return RobotState.AIMING;
    }
    
    // Left trigger (INTAKING) has fourth priority
    if (controller.leftTrigger().getAsBoolean()) {
      return RobotState.INTAKING;
    }
    
    // Right bumper (FERRYING) has fifth priority
    if (controller.rightBumper().getAsBoolean()) {
      return RobotState.FERRYING;
    }
    
    // Default to IDLE if no buttons pressed
    return RobotState.IDLE;
  }

  /** Gets the drive subsystem. */
  public Drive getDrive() {
    return drive;
  }

  /** Gets the vision subsystem. */
  public Vision getVision() {
    return vision;
  }

  /** Gets the intake subsystem. */
  public Intake getIntake() {
    return intake;
  }

  /** Gets the conveyor subsystem. */
  public Conveyor getConveyor() {
    return conveyor;
  }

  /** Gets the feeder subsystem. */
  public Feeder getFeeder() {
    return feeder;
  }

  /** Gets the hood subsystem. */
  public Hood getHood() {
    return hood;
  }

  /** Gets the drum subsystem. */
  public Drum getDrum() {
    return drum;
  }

  /** Gets the LED subsystem. */
  public LED getLEDs() {
    return leds;
  }

  /** Gets the state machine. */
  public MasterStateMachine getStateMachine() {
    return stateMachine;
  }

  /** Gets the state commands. */
  public StateCommands getStateCommands() {
    return stateCommands;
  }

  /** Gets the controller. */
  public CommandXboxController getController() {
    return controller;
  }

  /** Gets the enhanced shooting controller. */
  public EnhancedShootingController getEnhancedShootingController() {
    return enhancedShootingController;
  }

  /** Gets the current robot state. */
  @AutoLogOutput
  public String getCurrentState() {
    return stateCommands.getCurrentState().toString();
  }

  /** Checks if currently aiming. */
  @AutoLogOutput
  public boolean isAiming() {
    return stateCommands.isAiming();
  }

  /** Checks if ready to shoot. */
  @AutoLogOutput
  public boolean isReadyToShoot() {
    return stateCommands.isReadyToShoot();
  }

  // --- Auto Code ---

  /** Auto chooser */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Set up auto routines */
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

  /** Register named command */
  private void registerNamedCommand(String name, Command command) {
    NamedCommands.registerCommand(name, safeCommand(command));
  }

  /** Safe command wrapper */
  private Command safeCommand(Command command) {
    if (command == null) {
      DriverStation.reportError(
          "Named command resolved to null. Using Commands.none() fallback.", false);
      return Commands.none();
    }
    return command;
  }

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

   @AutoLogOutput(key = "Robot/Hub Active")
    public boolean isHubActive() {
    // Updated for 130s teleop: first 10s both active, then 100s alternating 25s periods,
    // last 30s both active. Alternating relative to elapsed 10s start.
    double elapsed = Constants.GeneralConstants.matchTimer.get();

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