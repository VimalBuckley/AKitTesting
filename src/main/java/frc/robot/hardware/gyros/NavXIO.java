package frc.robot.hardware.gyros;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Consumer;

public class NavXIO extends GyroIO {
  private AHRS gyro;
  private double simAngle;
  private double simVelocity;

  public NavXIO(NavXComType connectionType) {
    this(connectionType, ahrs -> {});
  }

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
