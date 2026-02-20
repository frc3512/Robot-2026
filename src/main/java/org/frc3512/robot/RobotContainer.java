package org.frc3512.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc3512.robot.commands.DriveCommands;
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

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final Conveyor conveyor;
  private final Feeder feeder;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  ;

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

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
        // new VisionIOPhotonVision(
        //     VisionConstants.rearCamera, VisionConstants.robotToRear));

        flywheel = new Flywheel(new FlywheelIO_REAL());
        intake = new Intake(new IntakeIO_REAL());
        conveyor = new Conveyor(new ConveyorIO_REAL());
        hood = new Hood(new HoodIO_REAL());
        feeder = new Feeder(new FeederIO_REAL());

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

        break;
    }

    autoChooser = AutoBuilder.buildAutoChooser();

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

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when left stick is pressed
    controller.leftStick().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when right stick is pressed
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
