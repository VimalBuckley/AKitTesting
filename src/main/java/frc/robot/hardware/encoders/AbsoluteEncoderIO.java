package frc.robot.hardware.encoders;

import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** A class used to interface with absolute encoders */
public abstract class AbsoluteEncoderIO implements IOLayer, Loggable {
  private AbsoluteEncoderIOInputsAutoLogged inputs;

  /** Creates a new {@link AbsoluteEncoderIO} */
  public AbsoluteEncoderIO() {
    inputs = new AbsoluteEncoderIOInputsAutoLogged();
    Robot.ios.add(this);
  }

  @AutoLog
  public static class AbsoluteEncoderIOInputs {
    public double position;
  }

  /**
   * @return The "current" position of the absolute encoder, in radians
   */
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

  /**
   * Updates the given inputs to contain the encoder's current state. This should really only be
   * used for sim purposes to allow for multiple sim runs per loop, as using it breaks the IO layer
   * contract, thus breaks replaying for the surrounding code
   *
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(AbsoluteEncoderIOInputs inputs);

  /**
   * Steps the encoder's simulation forward dt seconds. The encoder will use the provided inputs
   *
   * @param velocity The velocity of the encoder at the end of the timestep, in radians/second
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for simulation
   */
  public abstract void updateSim(double velocity, double dt, AbsoluteEncoderIOInputs inputs);
}
