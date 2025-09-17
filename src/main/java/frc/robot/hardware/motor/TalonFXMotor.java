package frc.robot.hardware.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class TalonFXMotor implements MotorIO {
  private TalonFX motor;
  private DCMotorModel model;
  private MotorIOInputsAutoLogged inputs;
  private TorqueCurrentFOC currentRequest;

  public TalonFXMotor(int deviceID, CANBus canbus, Consumer<TalonFX> config, DCMotorModel model) {
    motor = new TalonFX(deviceID, canbus);
    config.accept(motor);
    this.model = model;
    currentRequest = new TorqueCurrentFOC(0);
  }

  public TalonFXMotor(int deviceID, Consumer<TalonFX> config, DCMotorModel model) {
    this(deviceID, new CANBus(""), config, model);
  }

  public TalonFXMotor(
      int deviceID, CANBus canbus, TalonFXConfiguration config, DCMotorModel model) {
    this(
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

  public TalonFXMotor(int deviceID, TalonFXConfiguration config, DCMotorModel model) {
    this(deviceID, new CANBus(""), config, model);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setCurrent(double amps) {
    if (motor.getIsProLicensed().getValue()) {
      motor.setControl(currentRequest.withOutput(amps));
    } else {
      setVoltage(model.getVoltage(amps, inputs.velocity, inputs.statorVoltage));
    }
  }

  @Override
  public DCMotorModel getModel() {
    return model;
  }

  @Override
  public double getPosition() {
    return inputs.position;
  }

  @Override
  public double getVelocity() {
    return inputs.velocity;
  }

  @Override
  public double getTorque() {
    return model.getTorque(inputs.statorCurrent, inputs.velocity);
  }

  @Override
  public void setPosition(double newValue) {
    motor.setPosition(newValue);
  }

  @Override
  public void updateSim(double acceleration, double dt) {
    TalonFXSimState simState = motor.getSimState();
    double vel = acceleration * dt + inputs.velocity;
    double pos = 0.5 * (vel + inputs.velocity) * dt;
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simState.setRawRotorPosition(Units.radiansToRotations(pos));
    simState.setRotorVelocity(Units.radiansToRotations(vel));
  }

  @Override
  public void updateInputs(String name) {
    inputs.statorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    Logger.processInputs(name, inputs);
  }
}
