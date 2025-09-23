package frc.robot.hardware.encoders;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Consumer;

public class AnalogEncoderIO extends AbsoluteEncoderIO {
  private AnalogEncoder encoder;
  private double offset;
  private double simPosition;

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
      inputs.position = encoder.get() - offset;
    } else {
      inputs.position = simPosition;
    }
  }
}
