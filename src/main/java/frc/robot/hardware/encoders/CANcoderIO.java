package frc.robot.hardware.encoders;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import java.util.function.Consumer;

/** A class that wraps a {@link CANcoder} in an IO layer through {@link AbsoluteEncoderIO} */
public class CANcoderIO extends AbsoluteEncoderIO {
  private CANcoder encoder;

  /**
   * Creates a new {@link CANcoder}
   *
   * @param deviceID The CAN ID of the {@link CANcoder}
   * @param canbus The canbus encoder is connected to
   * @param config A method that takes in a {@link CANcoder}, and configures it as the caller sees
   *     fit. Most of the config should also apply during simulation
   */
  public CANcoderIO(int deviceID, CANBus canbus, Consumer<CANcoder> config) {
    super();
    encoder = new CANcoder(deviceID, canbus);
    config.accept(encoder);
  }

  /**
   * Creates a new {@link CANcoder}
   *
   * @param deviceID The CAN ID of the {@link CANcoder}
   * @param config A method that takes in a {@link CANcoder}, and configures it as the caller sees
   *     fit. Most of the config should also apply during simulation
   */
  public CANcoderIO(int deviceID, Consumer<CANcoder> config) {
    this(deviceID, new CANBus(), config);
  }

  /**
   * Creates a new {@link CANcoder}
   *
   * @param deviceID The CAN ID of the {@link CANcoder}
   * @param canbus The canbus encoder is connected to
   * @param config A {@link CANcoderConfiguration} that is applied to this {@link CANcoder}.
   *     Configuration is attempted 4 times before giving up. Most of the config should also apply
   *     during simulation
   */
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

  /**
   * Creates a new {@link CANcoder}
   *
   * @param deviceID The CAN ID of the {@link CANcoder}
   * @param config A {@link CANcoderConfiguration} that is applied to this {@link CANcoder}.
   *     Configuration is attempted 4 times before giving up. Most of the config should also apply
   *     during simulation
   */
  public CANcoderIO(int deviceID, CANcoderConfiguration config) {
    this(deviceID, new CANBus(), config);
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
