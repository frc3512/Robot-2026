package org.frc3512.robot.subsystems.shooter.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIO_REAL implements FeederIO {

    private TalonFX feeder;

    public FeederIO_REAL() {
        feeder = new TalonFX(FeederConstants.feederMotorID);

        feeder.getConfigurator().apply(FeederConstants.feeder);
    }

    @Override
    public void setFeeder(double speed) {
        feeder.set(speed);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Log velocity
        inputs.feederVelocity = feeder.getVelocity().getValueAsDouble() * 60.0;

        // Log applied volts
        inputs.feederAppliedOutput = feeder.getStatorCurrent().getValueAsDouble();
    }
    
}
