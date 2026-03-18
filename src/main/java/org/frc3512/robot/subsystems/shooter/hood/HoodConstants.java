package org.frc3512.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodConstants {

  public static final int motorID = 19;

  public static final double kP = 80;
  public static final double kI = 1;
  public static final double kD = 0.0;

  public static final double positionTolerance = 1; // Degrees

  public static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(20)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(20)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(112.5 / 1.0));
}
