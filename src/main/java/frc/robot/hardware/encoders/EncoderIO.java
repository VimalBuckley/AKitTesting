package frc.robot.hardware.encoders;

import frc.robot.hardware.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public abstract class EncoderIO implements IOLayer {
  protected String name;
  protected EncoderIOInputsAutoLogged inputs;
  public EncoderIO(String name) {
    this.name = name;
    inputs = new EncoderIOInputsAutoLogged();
    updateInputs();
  }
  @AutoLog
  public static class EncoderIOInputs {
    public double position;
  }

  public double getPosition() {
    return inputs.position;
  }

  public abstract void setPosition(double newValue);
  public abstract void updateSim(double velocity, double dt);
}
