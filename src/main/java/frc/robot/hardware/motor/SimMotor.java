package frc.robot.hardware.motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class SimMotor extends SubsystemBase implements MotorIO {
  private double volts;
  private double appliedVolts;
  private DCMotorModel model;
  private MotorConfiguration config;
  private double position;
  private double velocity;
  private DoubleSupplier acceleration;

  public SimMotor(MotorConfiguration config, DCMotorModel model, DoubleSupplier acceleration) {
    applyConfig(config);
    this.model = model;
    this.acceleration = acceleration;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  public double getVoltage() {
    return appliedVolts;
  }

  @Override
  public DCMotorModel getModel() {
    return model;
  }

  @Override
  public void applyConfig(MotorConfiguration config) {
    this.config = config.clone();
  }

  @Override
  public MotorConfiguration getConfig() {
    return config.clone();
  }

  @Override
  public double getPosition() {
    return position;
  }

  @Override
  public double getVelocity() {
    return velocity;
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
  }

  @Override
  public void periodic() {
    appliedVolts =
        model.getCurrentLimitedVoltage(
            config.statorCurrentLimit, config.supplyCurrentLimit, velocity, volts);
    double oldVelocity = velocity;
    velocity += 0.02 * acceleration.getAsDouble();
    position += 0.02 * (velocity + oldVelocity) / 2;
  }
}
