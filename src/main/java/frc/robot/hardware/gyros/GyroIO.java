package frc.robot.hardware.gyros;

import frc.robot.hardware.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public abstract class GyroIO implements IOLayer {
  protected String name;
  protected GyroIOInputsAutoLogged inputs;

  public GyroIO(String name) {
    this.name = name;
    inputs = new GyroIOInputsAutoLogged();
    updateInputs();
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

  public abstract void updateSim(double velocity, double dt);
}
