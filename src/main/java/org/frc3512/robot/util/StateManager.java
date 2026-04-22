package org.frc3512.robot.util;

import java.util.function.DoubleSupplier;

import org.frc3512.robot.commands.teleop.Aim;
import org.frc3512.robot.subsystems.conveyor.Conveyor;
import org.frc3512.robot.subsystems.conveyor.ConveyorStates;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.shooter.drum.Flywheel;
import org.frc3512.robot.subsystems.shooter.drum.FlywheelStates;
import org.frc3512.robot.subsystems.shooter.feeder.Feeder;
import org.frc3512.robot.subsystems.shooter.feeder.FeederStates;
import org.frc3512.robot.subsystems.shooter.hood.Hood;
import org.frc3512.robot.subsystems.shooter.hood.HoodStates;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase {
    private Aim aim;

    public enum RobotState {
        HOME,
        IDLE,
        INTAKING,
        AIMING,
        SHOOTING,
        DUMPING,
        FERRY
    }

    private RobotState currentState = RobotState.HOME;
    private RobotState wantedState = RobotState.HOME;
    private boolean aimCommandScheduled = false;

    private final Conveyor conveyor;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final Hood hood;
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public StateManager(Conveyor conveyor, Intake intake, Flywheel flywheel, 
                       Feeder feeder, Hood hood, Drive drive,
                       DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.flywheel = flywheel;
        this.feeder = feeder;
        this.hood = hood;
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        
        initializeAiming();
    }

    private void initializeAiming() {
        aim = new Aim(drive, xSupplier, ySupplier);
    }

    private void resetAimCommand() {
        if (aimCommandScheduled && aim != null) {
            CommandScheduler.getInstance().cancel(aim);
            aimCommandScheduled = false;
        }
    }

    @Override
    public void periodic() {

        handleStateTransitions();
        applyStates();

    }

    public void setWantedState(RobotState state) {
        wantedState = state;
    }

    private void handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                currentState = RobotState.HOME;
                resetAimCommand();
                break;
            case IDLE:
                currentState = RobotState.IDLE;
                resetAimCommand();
                break;
            case INTAKING:
                currentState = RobotState.INTAKING;
                resetAimCommand();
                break;
            case AIMING:
                currentState = RobotState.AIMING;
                break;
            case SHOOTING:
                if (aim != null && flywheel != null && aim.isAimed() && flywheel.isAtSetpoint()) {
                    currentState = RobotState.SHOOTING;
                } else {
                    currentState = RobotState.AIMING;
                }
                break;
            case DUMPING:
                if (aim != null && flywheel != null && aim.isAimed() && flywheel.isAtSetpoint()) {
                    currentState = RobotState.DUMPING;
                } else {
                    currentState = RobotState.AIMING;
                }
                break;
            case FERRY:
                currentState = RobotState.FERRY;
                resetAimCommand();
                break;
        }
    }
    
    private void applyStates() {
        switch (currentState) {
            case HOME:
                home();
                break;
            case IDLE:
                hold();
                break;
            case INTAKING:
                intake();
                break;
            case AIMING:
                aim();
                break;
            case SHOOTING:
                shoot();
                break;
            case DUMPING:
                dump();
                break;
            case FERRY:
                ferry();
                break;
        }
    }

    private void home() {
        intake.setWantedState(IntakeStates.HOME);
        conveyor.setWantedState(ConveyorStates.STOPPED);
        feeder.setWantedState(FeederStates.STOPPED);

        hood.setWantedState(HoodStates.HOME);
        flywheel.setWantedState(FlywheelStates.OFF);
    }

    private void hold() {
        intake.setWantedState(IntakeStates.IDLE);
        conveyor.setWantedState(ConveyorStates.STOPPED);
        feeder.setWantedState(FeederStates.STOPPED);

        hood.setWantedState(HoodStates.HOME);
        flywheel.setWantedState(FlywheelStates.IDLE);
    }

    private void intake() {
        intake.setWantedState(IntakeStates.INTAKING);
        conveyor.setWantedState(ConveyorStates.STOPPED);
        feeder.setWantedState(FeederStates.STOPPED);

        flywheel.setWantedState(FlywheelStates.IDLE);
        hood.setWantedState(HoodStates.HOME);
    }

    private void shoot() {
        intake.setWantedState(IntakeStates.COMPRESSING);
        conveyor.setWantedState(ConveyorStates.FEEDING);
        feeder.setWantedState(FeederStates.FEEDING);
    }

    private void dump() {
        intake.setWantedState(IntakeStates.INTAKING);
        conveyor.setWantedState(ConveyorStates.FEEDING);
        feeder.setWantedState(FeederStates.FEEDING);
    }

    private void ferry() {
        intake.setWantedState(IntakeStates.COMPRESSING);
        conveyor.setWantedState(ConveyorStates.FEEDING);
        feeder.setWantedState(FeederStates.FEEDING);

        // Set specific ferry configuration - FERRY state doesn't exist in enums
        flywheel.setTargetRPM(3250);
        hood.setPosition(40.0);
    }

    public void aim() {
        // Only schedule/cancel aim command when state changes to avoid race conditions
        if (!aimCommandScheduled) {
            // Cancel any existing aim command to avoid conflicts
            if (CommandScheduler.getInstance().isScheduled(aim)) {
                CommandScheduler.getInstance().cancel(aim);
            }
            CommandScheduler.getInstance().schedule(aim);
            aimCommandScheduled = true;
        }
        
        // Update Subsystem states
        intake.setWantedState(IntakeStates.IDLE);
        conveyor.setWantedState(ConveyorStates.STOPPED);
        feeder.setWantedState(FeederStates.STOPPED);

        if (aim != null) {
            hood.setPosition(aim.getAngle());
            flywheel.setTargetRPM(aim.getRPM());
        }
    }
}
