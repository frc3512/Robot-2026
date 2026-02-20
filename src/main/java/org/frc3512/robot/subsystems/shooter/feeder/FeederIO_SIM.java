package org.frc3512.robot.subsystems.shooter.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIO_SIM implements FeederIO {

    private DCMotorSim feederSim;
    private double feederSpeed = 0.0;

    public FeederIO_SIM() {
    feederSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));    }

    @Override
    public void setFeeder(double speed) {
        feederSpeed = speed;
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederSim.setInputVoltage(feederSpeed * 12.0);
        feederSim.update(0.02);

        inputs.feederVelocity = feederSim.getAngularVelocityRPM();
        inputs.feederAppliedOutput = feederSpeed * 12.0;
    }
    
}
