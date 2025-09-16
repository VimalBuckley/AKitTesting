package frc.robot.hardware.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class SparkMaxMotor extends SubsystemBase implements MotorIO {
  private SparkMax motor;
  private MotorConfiguration config;
  private DCMotorModel model;
  private double volts;

  public SparkMaxMotor(int canID, MotorConfiguration config, DCMotorModel model) {
    motor = new SparkMax(canID, MotorType.kBrushless);
    applyConfig(config);
    this.model = model;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public DCMotorModel getModel() {
    return model;
  }

  @Override
  public void applyConfig(MotorConfiguration config) {
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    config.statorCurrentLimit.ifPresent(sparkConfig::smartCurrentLimit);
    sparkConfig.inverted(config.inverted);
    sparkConfig.idleMode(config.brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    this.config = config.clone();
  }

  @Override
  public MotorConfiguration getConfig() {
    return config.clone();
  }

  @Override
  public double getPosition() {
    return Units.rotationsToRadians(motor.getEncoder().getPosition());
  }

  @Override
  public double getVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
  }

  @Override
  public void setPosition(double position) {
    motor.getEncoder().setPosition(Units.radiansToRotations(position));
  }

  @Override
  public void periodic() {
    double appliedVolts =
        model.getCurrentLimitedVoltage(
            Optional.empty(), // Spark maxes have a stator limit, so we can let them do that part
            config.supplyCurrentLimit,
            getVelocity(),
            volts);
    motor.setVoltage(appliedVolts);
  }
}
