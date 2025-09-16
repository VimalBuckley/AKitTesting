package frc.robot.hardware.motor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFXMotor extends SubsystemBase implements MotorIO {
  private TalonFX motor;
  private MotorConfiguration config;
  private DCMotorModel model;
  private double volts;

  public TalonFXMotor(int canID, MotorConfiguration config, DCMotorModel model) {
    this(canID, "", config, model);
  }

  public TalonFXMotor(int canID, String canbus, MotorConfiguration config, DCMotorModel model) {
    motor = new TalonFX(canID, canbus);
    applyConfig(config);
    this.model = model;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public DCMotorModel getModel() {
    return model;
  }

  @Override
  public void applyConfig(MotorConfiguration config) {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorCurrentLimit.isPresent();
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit.orElse(0);
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorCurrentLimit.isPresent();
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit.orElse(0);
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyCurrentLimit.isPresent();
    talonConfig.Voltage.PeakReverseVoltage = config.maxNegativeVoltage;
    talonConfig.Voltage.PeakForwardVoltage = config.maxPositiveVoltage;
    talonConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode =
        config.brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    StatusCode code = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5 && code != StatusCode.OK; i++) {
      code = motor.getConfigurator().apply(talonConfig);
    }
    this.config = config.clone();
  }

  @Override
  public MotorConfiguration getConfig() {
    return config.clone();
  }

  @Override
  public double getPosition() {
    return Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
  }

  @Override
  public double getVelocity() {
    return Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void setPosition(double newValue) {
    motor.setPosition(Units.radiansToRotations(newValue));
  }

  @Override
  public void periodic() {
    motor.setVoltage(volts);
  }
}
