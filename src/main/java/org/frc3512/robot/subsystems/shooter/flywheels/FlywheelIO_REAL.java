package org.frc3512.robot.subsystems.shooter.flywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.List;

// Credit to the 2026 WCP CC for the majority of this code
public class FlywheelIO_REAL implements FlywheelIO {
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

  private final TalonFX leftMotor, middleMotor, rightMotor;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public FlywheelIO_REAL() {
    leftMotor = new TalonFX(FlywheelConstants.leftMotorID);
    middleMotor = new TalonFX(FlywheelConstants.middleMotorID);
    rightMotor = new TalonFX(FlywheelConstants.rightMotorID);

    motors = List.of(leftMotor, middleMotor, rightMotor);

    configureMotor(leftMotor, InvertedValue.Clockwise_Positive);
    configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);
  }

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(
                        12.0
                            / RPM.of(6000)
                                .in(RotationsPerSecond)) // 12 volts when requesting max RPS
                );

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setRPM(double rpm) {
    for (final TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    for (final TalonFX motor : motors) {
      motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
    }
  }

  @Override
  public void stop() {
    setPercentOutput(0.0);
  }

  @Override
  public boolean isVelocityWithinTolerance() {
    return motors.stream()
        .allMatch(
            motor -> {
              final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
              final AngularVelocity currentVelocity = motor.getVelocity().getValue();
              final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
              return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
            });
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.rpmSetpoint = velocityRequest.Velocity * 60.0;

    inputs.leftVelocity = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.middleVelocity = middleMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightVelocity = rightMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.leftAppliedVolts = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.middleAppliedVolts = middleMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightAppliedVolts = rightMotor.getStatorCurrent().getValueAsDouble();

    inputs.isVelocityWithinTolerance = isVelocityWithinTolerance();
  }
}
