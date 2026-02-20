package org.frc3512.robot.subsystems.conveyor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ConveyorIO_SIM implements ConveyorIO {

  private DCMotorSim hopperSim;
  private double hopperSpeed = 0.0;

  public ConveyorIO_SIM() {
    hopperSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    hopperSim.setInputVoltage(hopperSpeed * 12.0);
    hopperSim.update(0.02);
    inputs.hopperVelocity = hopperSim.getAngularVelocityRPM();
    inputs.hopperVolts = hopperSpeed * 12.0;
  }

  @Override
  public void setHopper(double speed) {
    hopperSpeed = speed;
  }
}
