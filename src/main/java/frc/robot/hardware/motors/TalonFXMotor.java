package frc.robot.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class TalonFXMotor extends MotorIO {
  private TalonFX motor;

  public TalonFXMotor(
      String name, int deviceID, CANBus canbus, Consumer<TalonFX> config, DCMotor model) {
    super(name, model);
    motor = new TalonFX(deviceID, canbus);
    config.accept(motor);
  }

  public TalonFXMotor(String name, int deviceID, Consumer<TalonFX> config, DCMotor model) {
    this(name, deviceID, new CANBus(""), config, model);
  }

  public TalonFXMotor(
      String name, int deviceID, CANBus canbus, TalonFXConfiguration config, DCMotor model) {
    this(
        name,
        deviceID,
        canbus,
        fx -> {
          StatusCode status = StatusCode.StatusCodeNotInitialized;
          for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
            status = fx.getConfigurator().apply(config);
          }
        },
        model);
  }

  public TalonFXMotor(String name, int deviceID, TalonFXConfiguration config, DCMotor model) {
    this(name, deviceID, new CANBus(""), config, model);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double newValue) {
    motor.setPosition(newValue);
  }

  @Override
  public void updateSim(double acceleration, double dt) {
    TalonFXSimState simState = motor.getSimState();
    double vel = acceleration * dt + inputs.velocity;
    double pos = inputs.position + 0.5 * (vel + inputs.velocity) * dt;
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simState.setRawRotorPosition(Units.radiansToRotations(pos));
    simState.setRotorVelocity(Units.radiansToRotations(vel));
  }

  @Override
  public void updateInputs() {
    inputs.statorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
    inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
    Logger.processInputs(name, inputs);
  }
}
