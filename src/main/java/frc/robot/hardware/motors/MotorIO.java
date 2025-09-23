package frc.robot.hardware.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class MotorIO implements Loggable, IOLayer {
  private DCMotor model;
  private MotorIOInputsAutoLogged inputs;

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

  public MotorIO(DCMotor model) {
    this.model = model;
    inputs = new MotorIOInputsAutoLogged();
    Robot.ios.add(this);
  }

  public abstract void setVoltage(double volts);

  public abstract void setPosition(double newValue);

  public abstract void updateInputs(MotorIOInputs inputs);

  public abstract void updateSim(double velocity, double dt, MotorIOInputs inputs);

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

  @Override
  public void updateInputs() {
    updateInputs(inputs);
    if (RobotBase.isSimulation()) {
      Robot.supplyCurrents.add(inputs.supplyCurrent);
    }
  }

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }
}
