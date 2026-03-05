package org.frc3512.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIO_SIM implements ClimberIO {

  private DCMotorSim climberSim;
  private double climberSpeed = 0.0;

  public ClimberIO_SIM() {
    climberSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.setInputVoltage(climberSpeed * 12.0);
    climberSim.update(0.02);
    inputs.climberVelocity = climberSim.getAngularVelocityRPM();
    inputs.climberVolts = climberSpeed * 12.0;
  }

  @Override
  public void setClimber(double speed) {
    climberSpeed = speed;
  }
}
