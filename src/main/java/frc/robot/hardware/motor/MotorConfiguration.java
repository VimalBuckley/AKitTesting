package frc.robot.hardware.motor;

import java.util.Optional;

public class MotorConfiguration implements Cloneable {
  public Optional<Integer> statorCurrentLimit = Optional.of(120);
  public Optional<Integer> supplyCurrentLimit = Optional.of(70);
  public double maxNegativeVoltage = -12;
  public double maxPositiveVoltage = 12;
  public boolean inverted = false;
  public boolean brakeModeEnabled = false;

  public record ConfigSupport(
      boolean supportsStatorLimit, boolean supportsSupplyLimit, boolean supportsVoltageRange) {}

  public MotorConfiguration withStatorCurrentLimit(Optional<Integer> limit) {
    statorCurrentLimit = limit;
    return this;
  }

  public MotorConfiguration withSupplyCurrentLimit(Optional<Integer> limit) {
    supplyCurrentLimit = limit;
    return this;
  }

  public MotorConfiguration withMaxNegativeVoltage(double volts) {
    maxNegativeVoltage = volts;
    return this;
  }

  public MotorConfiguration withMaxPositiveVoltage(double volts) {
    maxPositiveVoltage = volts;
    return this;
  }

  public MotorConfiguration withInverted(boolean isInverted) {
    inverted = isInverted;
    return this;
  }

  public MotorConfiguration withBrakeModeEnabled(boolean enabled) {
    brakeModeEnabled = enabled;
    return this;
  }

  @Override
  public MotorConfiguration clone() {
    MotorConfiguration config = new MotorConfiguration();
    config.statorCurrentLimit.equals(statorCurrentLimit);
    config.supplyCurrentLimit.equals(supplyCurrentLimit);
    config.maxNegativeVoltage = maxNegativeVoltage;
    config.maxPositiveVoltage = maxPositiveVoltage;
    config.inverted = inverted;
    config.brakeModeEnabled = brakeModeEnabled;
    return config;
  }
}
