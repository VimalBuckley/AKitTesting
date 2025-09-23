package frc.robot.hardware.gyros;

import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class GyroIO implements IOLayer, Loggable {
  private GyroIOInputsAutoLogged inputs;

  public GyroIO() {
    inputs = new GyroIOInputsAutoLogged();
    Robot.ios.add(this);
  }

  @AutoLog
  public static class GyroIOInputs {
    public double angle;
    public double velocity;
    public boolean isConnected;
  }

  public double getAngle() {
    return inputs.angle;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  @Override
  public void updateInputs() {
    updateInputs(inputs);
  }

  public abstract void updateInputs(GyroIOInputs inputs);

  public abstract void updateSim(double velocity, double dt, GyroIOInputs inputs);

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }
}
