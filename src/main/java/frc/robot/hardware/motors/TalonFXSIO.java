package frc.robot.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Consumer;

public class TalonFXSIO extends MotorIO {
  private TalonFXS motor;

  public TalonFXSIO(int deviceID, CANBus canbus, Consumer<TalonFXS> config, DCMotor model) {
    super(model);
    motor = new TalonFXS(deviceID, canbus);
    config.accept(motor);
    updateInputs();
  }

  public TalonFXSIO(int deviceID, Consumer<TalonFXS> config, DCMotor model) {
    this(deviceID, new CANBus(), config, model);
  }

  public TalonFXSIO(int deviceID, CANBus canbus, TalonFXSConfiguration config, DCMotor model) {
    this(
        deviceID,
        canbus,
        fxs -> {
          StatusCode status = StatusCode.StatusCodeNotInitialized;
          for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
            status = fxs.getConfigurator().apply(config);
          }
        },
        model);
  }

  public TalonFXSIO(int deviceID, TalonFXSConfiguration config, DCMotor model) {
    this(deviceID, new CANBus(), config, model);
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
  public void updateSim(double velocity, double dt, MotorIOInputs inputs) {
    TalonFXSSimState simState = motor.getSimState();
    double pos = inputs.position + 0.5 * (velocity + inputs.velocity) * dt;
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simState.setRawRotorPosition(Units.radiansToRotations(pos));
    simState.setRotorVelocity(Units.radiansToRotations(velocity));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.statorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
    inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
  }
}
