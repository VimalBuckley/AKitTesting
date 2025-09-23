package frc.robot.hardware.encoders;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import java.util.function.Consumer;

public class CANcoderIO extends AbsoluteEncoderIO {
  private CANcoder encoder;

  public CANcoderIO(int deviceID, Consumer<CANcoder> config) {
    this(deviceID, new CANBus(), config);
  }

  public CANcoderIO(int deviceID, CANBus canbus, Consumer<CANcoder> config) {
    super();
    encoder = new CANcoder(deviceID, canbus);
    config.accept(encoder);
  }

  public CANcoderIO(int deviceID, CANcoderConfiguration config) {
    this(deviceID, new CANBus(), config);
  }

  public CANcoderIO(int deviceID, CANBus canbus, CANcoderConfiguration config) {
    this(
        deviceID,
        canbus,
        CANcoder -> {
          StatusCode status = StatusCode.StatusCodeNotInitialized;
          for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
            CANcoder.getConfigurator().apply(config);
          }
        });
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.position = Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void updateSim(double velocity, double dt, AbsoluteEncoderIOInputs inputs) {
    encoder.getSimState().setRawPosition(Units.radiansToRotations(inputs.position + velocity * dt));
  }
}
