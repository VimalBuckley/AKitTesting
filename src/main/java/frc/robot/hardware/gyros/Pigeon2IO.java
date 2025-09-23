package frc.robot.hardware.gyros;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.util.Units;
import java.util.function.Consumer;

public class Pigeon2IO extends GyroIO {
  private Pigeon2 gyro;

  public Pigeon2IO(int deviceID, CANBus canbus, Consumer<Pigeon2> config) {
    super();
    gyro = new Pigeon2(deviceID, canbus);
    config.accept(gyro);
    updateInputs();
  }

  public Pigeon2IO(int deviceID, Consumer<Pigeon2> config) {
    this(deviceID, new CANBus(), config);
  }

  public Pigeon2IO(int deviceID, CANBus canbus, Pigeon2Configuration config) {
    this(
        deviceID,
        canbus,
        pigeon -> {
          StatusCode status = StatusCode.StatusCodeNotInitialized;
          for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
            pigeon.getConfigurator().apply(config);
          }
        });
  }

  public Pigeon2IO(int deviceID, Pigeon2Configuration config) {
    this(deviceID, new CANBus(), config);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.angle = Units.degreesToRadians(gyro.getYaw().getValueAsDouble());
    inputs.velocity = Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());
    inputs.isConnected = gyro.isConnected();
  }

  @Override
  public void updateSim(double velocity, double dt, GyroIOInputs inputs) {
    Pigeon2SimState simState = gyro.getSimState();
    simState.setAngularVelocityZ(Units.radiansToDegrees(velocity));
    simState.setRawYaw(Units.radiansToDegrees(inputs.angle + velocity * dt));
  }
}
