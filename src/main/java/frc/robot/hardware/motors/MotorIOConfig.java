package frc.robot.hardware.motors;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorIOConfig {
  public int statorLimit = 120;
  public int supplyLimit = 70;
  public InvertedValue inversion = InvertedValue.CounterClockwise_Positive;
  public NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public double maxPositiveVoltage = 12;
  public double maxNegativeVoltage = -12;

  public MotorIOConfig withStatorLimit(int statorLimit) {
    this.statorLimit = statorLimit;
    return this;
  }

  public MotorIOConfig withSupplyLimit(int supplyLimit) {
    this.supplyLimit = supplyLimit;
    return this;
  }

  public MotorIOConfig withInversion(InvertedValue inversion) {
    this.inversion = inversion;
    return this;
  }

  public MotorIOConfig withNeutralMode(NeutralModeValue neutralMode) {
    this.neutralMode = neutralMode;
    return this;
  }

  public MotorIOConfig withOutputRange(double maxNegativeVoltage, double maxPositiveVoltage) {
    this.maxNegativeVoltage = maxNegativeVoltage;
    this.maxPositiveVoltage = maxPositiveVoltage;
    return this;
  }

  public void copyTo(MotorIOConfig config) {
    config.statorLimit = statorLimit;
    config.supplyLimit = supplyLimit;
    config.inversion = inversion;
    config.neutralMode = neutralMode;
    config.maxPositiveVoltage = maxPositiveVoltage;
    config.maxNegativeVoltage = maxNegativeVoltage;
  }
}
