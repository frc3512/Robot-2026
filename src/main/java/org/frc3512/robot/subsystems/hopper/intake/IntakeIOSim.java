package org.frc3512.robot.subsystems.hopper.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of intake IO.
 * Simulation is always based on voltage control.
 */
public class IntakeIOSim implements IntakeIO {
  private static final double ROLLER_KP = 0.1;
  private static final double ROLLER_KV = 0.12;
  private static final double EXTENSION_KP = 2.0;
  private static final double EXTENSION_KD = 0.1;
  private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor EXTENSION_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim roller1Sim;
  private final DCMotorSim roller2Sim;
  private final DCMotorSim extensionSim;

  private boolean rollerClosedLoop = false;
  private boolean extensionClosedLoop = false;
  private PIDController rollerController = new PIDController(ROLLER_KP, 0, 0);
  private PIDController extensionController = new PIDController(EXTENSION_KP, 0, EXTENSION_KD);
  private double rollerFFVolts = 0.0;
  private double roller1AppliedVolts = 0.0;
  private double roller2AppliedVolts = 0.0;
  private double extensionAppliedVolts = 0.0;

  public IntakeIOSim() {
    // Create motor sims
    roller1Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, 0.01, 1.0),
        ROLLER_GEARBOX);
    roller2Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, 0.01, 1.0),
        ROLLER_GEARBOX);
    extensionSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(EXTENSION_GEARBOX, 0.01, 1.0),
        EXTENSION_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update roller 1
    inputs.roller1Connected = true;
    inputs.roller1VelocityRPS = roller1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.roller1AppliedVolts = roller1AppliedVolts;
    inputs.roller1CurrentAmps = roller1Sim.getCurrentDrawAmps();
    inputs.roller1TempCelsius = 25.0; // Simulated temperature

    // Update roller 2
    inputs.roller2Connected = true;
    inputs.roller2VelocityRPS = roller2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.roller2AppliedVolts = roller2AppliedVolts;
    inputs.roller2CurrentAmps = roller2Sim.getCurrentDrawAmps();
    inputs.roller2TempCelsius = 25.0; // Simulated temperature

    // Update extension
    inputs.extensionConnected = true;
    inputs.extensionPositionInches = rotationsToInches(extensionSim.getAngularPositionRotations());
    inputs.extensionVelocityInchesPerSec = rpsToInchesPerSec(extensionSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionCurrentAmps = extensionSim.getCurrentDrawAmps();
    inputs.extensionTempCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setRollerVelocity(double mechanismRPM) {
    rollerClosedLoop = true;
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Intake.ROLLER_MOTOR_RPM_PER_MECHANISM_RPM;
    rollerFFVolts = ROLLER_KV * motorRPS;
    rollerController.setSetpoint(motorRPS);
  }

  @Override
  public void setRollerOpenLoop(double output) {
    rollerClosedLoop = false;
    roller1AppliedVolts = output;
    roller2AppliedVolts = output;
  }

  @Override
  public void setExtensionPosition(double positionInches) {
    extensionClosedLoop = true;
    double motorRotations = inchesToRotations(positionInches);
    extensionController.setSetpoint(motorRotations);
  }

  @Override
  public void setExtensionOpenLoop(double output) {
    extensionClosedLoop = false;
    extensionAppliedVolts = output;
  }

  @Override
  public void stop() {
    rollerClosedLoop = false;
    extensionClosedLoop = false;
    roller1AppliedVolts = 0.0;
    roller2AppliedVolts = 0.0;
    extensionAppliedVolts = 0.0;
  }

  @Override
  public void setCurrentLimits(double currentLimitAmps) {
    // In simulation, current limits are not enforced on the simulated motors
    // This method exists for interface compatibility
  }

  /** Runs the simulation, updating all motor states. */
  public void simulate(double dtSeconds, double supplyVoltage) {
    // Update roller motors
    if (rollerClosedLoop) {
      double roller1Volts = MathUtil.clamp(
          rollerFFVolts + rollerController.calculate(roller1Sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      double roller2Volts = MathUtil.clamp(
          rollerFFVolts + rollerController.calculate(roller2Sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      roller1AppliedVolts = roller1Volts;
      roller2AppliedVolts = roller2Volts;
      roller1Sim.setInputVoltage(roller1Volts);
      roller2Sim.setInputVoltage(roller2Volts);
    } else {
      roller1Sim.setInputVoltage(roller1AppliedVolts);
      roller2Sim.setInputVoltage(roller2AppliedVolts);
    }

    // Update extension motor
    if (extensionClosedLoop) {
      double extensionVolts = MathUtil.clamp(
          extensionController.calculate(extensionSim.getAngularPositionRotations()), -12.0, 12.0);
      extensionAppliedVolts = extensionVolts;
      extensionSim.setInputVoltage(extensionVolts);
    } else {
      extensionSim.setInputVoltage(extensionAppliedVolts);
    }

    // Update all simulations
    roller1Sim.update(dtSeconds);
    roller2Sim.update(dtSeconds);
    extensionSim.update(dtSeconds);
  }

  // Helper methods for position conversion
  private double rotationsToInches(double rotations) {
    return rotations * 2.0 * Math.PI * 1.0; // TODO: Update with actual drum radius
  }

  private double inchesToRotations(double inches) {
    return inches / (2.0 * Math.PI * 1.0); // TODO: Update with actual drum radius
  }

  private double rpsToInchesPerSec(double rps) {
    return rps * 2.0 * Math.PI * 1.0; // TODO: Update with actual drum radius
  }
}
