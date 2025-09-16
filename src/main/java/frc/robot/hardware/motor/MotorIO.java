package frc.robot.hardware.motor;

public interface MotorIO {
  public void setVoltage(double volts);

  public double getVoltage();

  public DCMotorModel getModel();

  public void applyConfig(MotorConfiguration config);

  public MotorConfiguration getConfig();

  public default double getTorque() {
    DCMotorModel model = getModel();
    MotorConfiguration config = getConfig();
    if (!config.brakeModeEnabled && getVoltage() == 0) {
      return 0;
    }
    return model.getTorque(
        config.statorCurrentLimit, config.supplyCurrentLimit, getVelocity(), getVoltage());
  }

  public double getPosition();

  public double getVelocity();

  public void setPosition(double position);
}
