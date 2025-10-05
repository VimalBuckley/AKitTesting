package frc.robot.hardware.encoders;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Consumer;

/** A class that wraps a {@link AnalongEncoder} in an IO layer through {@link AbsoluteEncoderIO} */
public class AnalogEncoderIO extends AbsoluteEncoderIO {
  private AnalogEncoder encoder;
  private double offset;
  private double simPosition;

  /**
   * Creates a new {@link AnalongEncoder}
   *
   * @param channel The analog port this encoder is plugged into
   * @param offset The reading on the encoder, in radians, when its position should be zero
   * @param config A method that takes in a {@link AnalogEncoder} and configures it as the caller
   *     sees fit
   */
  public AnalogEncoderIO(int channel, double offset, Consumer<AnalogEncoder> config) {
    super();
    encoder = new AnalogEncoder(channel);
    config.accept(encoder);
    this.offset = offset;
    updateInputs();
  }

  @Override
  public void updateSim(double velocity, double dt, AbsoluteEncoderIOInputs inputs) {
    simPosition = inputs.position + velocity * dt;
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    if (RobotBase.isReal()) {
      inputs.position = Units.rotationsToRadians(encoder.get()) - offset;
    } else {
      inputs.position = simPosition;
    }
  }
}
