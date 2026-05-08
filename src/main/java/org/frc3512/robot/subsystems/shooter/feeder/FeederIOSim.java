package org.frc3512.robot.subsystems.shooter.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of feeder IO.
 * Simulation is always based on voltage control.
 */
public class FeederIOSim implements FeederIO {
  private static final double KP = 0.1;
  private static final double KV = 0.12;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim feedMotorSim;
  private final DCMotorSim boosterMotorSim;

  private boolean feedClosedLoop = false;
  private boolean boosterClosedLoop = false;
  private PIDController feedController = new PIDController(KP, 0, 0);
  private PIDController boosterController = new PIDController(KP, 0, 0);
  private double feedFFVolts = 0.0;
  private double boosterFFVolts = 0.0;
  private double feedAppliedVolts = 0.0;
  private double boosterAppliedVolts = 0.0;

  public FeederIOSim() {
    // Create motor sims
    feedMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
    boosterMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Update feed motor
    inputs.feedConnected = true;
    inputs.feedVelocityRPS = feedMotorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.feedAppliedVolts = feedAppliedVolts;
    inputs.feedCurrentAmps = feedMotorSim.getCurrentDrawAmps();
    inputs.feedTempCelsius = 25.0; // Simulated temperature

    // Update booster motor
    inputs.boosterConnected = true;
    inputs.boosterVelocityRPS = boosterMotorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.boosterAppliedVolts = boosterAppliedVolts;
    inputs.boosterCurrentAmps = boosterMotorSim.getCurrentDrawAmps();
    inputs.boosterTempCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setFeedVelocity(double mechanismRPM) {
    feedClosedLoop = true;
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
    feedFFVolts = KV * motorRPS;
    feedController.setSetpoint(motorRPS);
  }

  @Override
  public void setFeedOpenLoop(double output) {
    feedClosedLoop = false;
    feedAppliedVolts = output;
  }

  @Override
  public void setBoosterVelocity(double mechanismRPM) {
    boosterClosedLoop = true;
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
    boosterFFVolts = KV * motorRPS;
    boosterController.setSetpoint(motorRPS);
  }

  @Override
  public void setBoosterOpenLoop(double output) {
    boosterClosedLoop = false;
    boosterAppliedVolts = output;
  }

  @Override
  public void stop() {
    feedClosedLoop = false;
    boosterClosedLoop = false;
    feedAppliedVolts = 0.0;
    boosterAppliedVolts = 0.0;
  }

  @Override
  public void setCurrentLimits(double currentLimitAmps) {
    // In simulation, current limits are not enforced on simulated motors
    // This method exists for interface compatibility
  }

  /** Runs simulation, updating all motor states. */
  public void simulate(double dtSeconds, double supplyVoltage) {
    // Update feed motor
    if (feedClosedLoop) {
      double feedVolts = MathUtil.clamp(
          feedFFVolts + feedController.calculate(feedMotorSim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      feedAppliedVolts = feedVolts;
      feedMotorSim.setInputVoltage(feedVolts);
    } else {
      feedMotorSim.setInputVoltage(feedAppliedVolts);
    }

    // Update booster motor
    if (boosterClosedLoop) {
      double boosterVolts = MathUtil.clamp(
          boosterFFVolts + boosterController.calculate(boosterMotorSim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      boosterAppliedVolts = boosterVolts;
      boosterMotorSim.setInputVoltage(boosterVolts);
    } else {
      boosterMotorSim.setInputVoltage(boosterAppliedVolts);
    }

    // Update all simulations
    feedMotorSim.update(dtSeconds);
    boosterMotorSim.update(dtSeconds);
  }
}
