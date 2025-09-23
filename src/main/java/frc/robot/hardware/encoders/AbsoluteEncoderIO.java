package frc.robot.hardware.encoders;

import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class AbsoluteEncoderIO implements IOLayer, Loggable {
  private AbsoluteEncoderIOInputsAutoLogged inputs;

  public AbsoluteEncoderIO() {
    inputs = new AbsoluteEncoderIOInputsAutoLogged();
    Robot.ios.add(this);
  }

  @AutoLog
  public static class AbsoluteEncoderIOInputs {
    public double position;
  }

  public double getPosition() {
    return inputs.position;
  }

  @Override
  public void updateInputs() {
    updateInputs(inputs);
  }

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }

  public abstract void updateInputs(AbsoluteEncoderIOInputs inputs);

  public abstract void updateSim(double velocity, double dt, AbsoluteEncoderIOInputs inputs);
}
