package org.frc3512.robot.subsystems;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeConstants.IntakeState;
import org.frc3512.robot.subsystems.conveyor.Conveyor;
import org.frc3512.robot.subsystems.shooter.drum.Flywheel;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.commands.teleop.ShootAndMove;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {
    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Hood hood;
    
    // Joystick suppliers for ShootAndMove
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    
    // State management
    public States currentState = States.HOMED;
    public States wantedState = States.HOMED;
    public States prevState = States.HOMED;
    
    @Override
    public void periodic() {
        prevState = currentState;
        Logger.recordOutput("Superstructure/PrevState", prevState.toString());
        Logger.recordOutput("Superstructure/CurrentState", currentState.toString());
        Logger.recordOutput("Superstructure/WantedState", wantedState.toString());
        
        handleStateTransitions();
        applyStates();
    }

    public Superstructure(Drive drive, Intake intake, Conveyor conveyor, Feeder feeder, Flywheel flywheel, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.drive = drive;
        this.intake = intake;
        this.conveyor = conveyor;
        this.feeder = feeder;
        this.flywheel = flywheel;
        this.hood = hood;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    public States getCurrentState() {
        return currentState;
    }

    public States getWantedState() {
        return wantedState;
    }

    public void setCurrentState(States state) {
        currentState = state;
    }

    public void setWantedState(States state) {
        wantedState = state;
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case HOMED:
                currentState = States.HOMED;
                break;
            case INTAKING:
                currentState = States.INTAKING;
                break;
            case IDLE:
                currentState = States.IDLE;
                break;
            case PREPPING_SHOT:
                currentState = States.PREPPING_SHOT;
                break;
            case SHOOTING:
                currentState = States.SHOOTING;
                break;
            case FERRYING:
                currentState = States.FERRYING;
                break;
            case DUMPING:
                currentState = States.DUMPING;
                break;
        }
    }

    public void applyStates() {
        switch (currentState) {
            case HOMED:
                stateHOMED();
                break;
            case INTAKING:
                stateINTAKING();
                break;
            case IDLE:
                stateIDLE();
                break;
            case PREPPING_SHOT:
                statePREPPING_SHOT();
                break;
            case SHOOTING:
                stateSHOOTING();
                break;
            case FERRYING:
                stateFERRYING();
                break;
            case DUMPING:
                stateDUMPING();
                break;
        }
    }

    private Command stateHOMED() {
        // Retract intake and stop all mechanisms
        return Commands.sequence(
            intake.setPosition(IntakeState.STOWED),
            intake.setRollerSpeed(0.0),
            conveyor.setHopper(0.0),
            feeder.setFeeder(0.0),
            flywheel.setRPM(0.0),
            hood.setPosition(0.0));
    }

    private Command stateIDLE() {
        // Set intake to extended but off, prepare shooter for idle
        return Commands.sequence(
            intake.setPosition(IntakeState.EXTEND),
            intake.setRollerSpeed(0.0),
            conveyor.setHopper(0.0),
            feeder.setFeeder(0.0),
            flywheel.setRPM(2800.0),
            hood.setPosition(0.0));
    }

    private Command statePREPPING_SHOT() {
        // Prepare shooter systems while keeping intake ready
        return Commands.sequence(
            intake.setPosition(IntakeState.EXTEND),
            intake.setRollerSpeed(0.0),
            conveyor.setHopper(0.0),
            feeder.setFeeder(0.0),
            flywheel.setRPM(3000.0),
            hood.setPosition(0.0));
    }

    private Command stateINTAKING() {
        // Extend and run intake, start conveyor after delay
        return Commands.sequence(
            intake.setPosition(IntakeState.EXTEND),
            intake.setRollerSpeed(0.70),
            conveyor.setHopper(0.2),
            feeder.setFeeder(0.0),
            flywheel.setRPM(2800.0),
            hood.setPosition(0.0));
    }
    
    private Command stateFERRYING() {
        // Setup for long-distance shooting
        return Commands.sequence(
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
        // Agitate intake repeatedly until command is cancelled
        Commands.sequence(
                intake.setPosition(IntakeState.EXTEND),
                Commands.waitSeconds(0.25),
                intake.setPosition(IntakeState.AGITATE),
                Commands.waitSeconds(0.25))
            .repeatedly());
    }

    private Command stateSHOOTING() {
        // Full shooting sequence using ShootAndMove command
        return new ShootAndMove(
            drive,
            flywheel,
            hood,
            conveyor,
            feeder,
            intake,
            xSupplier,
            ySupplier);
    }

    private Command stateDUMPING() {
        // TODO: Implement dumping command
        return Commands.none();
    }
}
