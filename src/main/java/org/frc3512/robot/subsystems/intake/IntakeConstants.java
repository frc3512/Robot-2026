package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

  public static final int rollerMotorID = 16;
  public static final int extensionMotorID = 15;

  public static final TalonFXConfiguration rollerMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true));

  public static final TalonFXConfiguration extensionMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(new Slot0Configs().withKP(12))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(30.0 / 11.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(30)
                  .withSupplyCurrentLimitEnable(true));

  public static enum IntakeState {
    // Position values in rotations of motor
    EXTEND(2.85),
    AGITATE(1.2),
    STOWED(0.0);

    public final double position;

    private IntakeState(double position) {
      this.position = position;
    }
  }
}
