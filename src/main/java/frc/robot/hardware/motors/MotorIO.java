package frc.robot.hardware.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public abstract class MotorIO implements IOLayer {
  protected String name;
  protected DCMotor model;
  protected MotorIOInputsAutoLogged inputs;

  @AutoLog
  public static class MotorIOInputs {
    public double statorVoltage;
    public double supplyVoltage;
    public double position;
    public double velocity;
    public double statorCurrent;
    public double supplyCurrent;
    public double temperature;
  }

  public MotorIO(String name, DCMotor model) {
    this.name = name;
    this.model = model;
    inputs = new MotorIOInputsAutoLogged();
    updateInputs();
  }

  public abstract void setVoltage(double volts);

  public abstract void setPosition(double newValue);

  public abstract void updateSim(double acceleration, double dt);

  public DCMotor getModel() {
    return model;
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getVoltage() {
    return inputs.statorVoltage;
  }
}
