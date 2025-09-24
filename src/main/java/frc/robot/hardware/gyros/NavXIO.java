package frc.robot.hardware.gyros;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Consumer;

/**
 * A class that wraps a {@link AHRS} in an IO layer through {@link GyroIO}
 */
public class NavXIO extends GyroIO {
  private AHRS gyro;
  private double simAngle;
  private double simVelocity;

  /**
   * Creates a new {@link AHRS}
   * @param connectionType How the NavX is connected to the robot
   */
  public NavXIO(NavXComType connectionType) {
    this(connectionType, ahrs -> {});
  }

  /**
   * Creates a new {@link AHRS}
   * @param connectionType How the NavX is connected to the robot
   * @param config A method that takes in a {@link AHRS}, and configures it as
   * the caller sees fit
   */
  public NavXIO(NavXComType connectionType, Consumer<AHRS> config) {
    super();
    gyro = new AHRS(connectionType);
    config.accept(gyro);
    updateInputs();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    if (RobotBase.isReal()) {
      inputs.angle = Units.degreesToRadians(-gyro.getAngle());
      inputs.velocity = Units.degreesToRadians(-gyro.getRate());
      inputs.isConnected = gyro.isConnected();
    } else {
      inputs.angle = simAngle;
      inputs.velocity = simVelocity;
      inputs.isConnected = true;
    }
  }

  @Override
  public void updateSim(double velocity, double dt, GyroIOInputs inputs) {
    simAngle = inputs.angle + 0.5 * (inputs.velocity + velocity) * dt;
    simVelocity = velocity;
  }
}
