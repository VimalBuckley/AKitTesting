package frc.robot.hardware.gyros;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class NavXIO extends GyroIO {
  private AHRS gyro;
  private double simAngle;
  private double simVelocity;

  public NavXIO(String name, NavXComType connectionType) {
    this(name, connectionType, ahrs -> {});
  }

  public NavXIO(String name, NavXComType connectionType, Consumer<AHRS> config) {
    super(name);
    gyro = new AHRS(connectionType);
    config.accept(gyro);
  }

  @Override
  public void updateInputs() {
    if (RobotBase.isReal()) {
      inputs.angle = Units.degreesToRadians(-gyro.getAngle());
      inputs.velocity = Units.degreesToRadians(-gyro.getRate());
      inputs.isConnected = gyro.isConnected();
    } else {
      inputs.angle = simAngle;
      inputs.velocity = simVelocity;
      inputs.isConnected = true;
    }
    Logger.processInputs(name, inputs);
  }

  @Override
  public void updateSim(double velocity, double dt) {
    simAngle += 0.5 * (inputs.velocity + velocity) * dt;
    simVelocity = velocity;
  }
}
