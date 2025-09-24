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

/**
 * A class that wraps a {@link TalonFX} in an IO layer through {@link MotorIO}
 */
public class TalonFXIO extends MotorIO {
  private TalonFX motor;

  /**
   * Creates a new {@link TalonFX}
   * @param deviceID The CAN ID of the {@link TalonFX} 
   * @param canbus The canbus this motor is connected to
   * @param config A method that takes in a {@link TalonFX}, and configures it as 
   * the caller sees fit. Most of the config should also apply during simulation
   * @param model A {@link DCMotor} that represents the motor that this TalonFX is controlling. Do not apply a
   * gear ratio to this object, as gear ratios are dealt with separately. Also, do not set the motor count to more than one.
   */
  public TalonFXIO(int deviceID, CANBus canbus, Consumer<TalonFX> config, DCMotor model) {
    super(model);
    motor = new TalonFX(deviceID, canbus);
    config.accept(motor);
    updateInputs();
  }

  /**
   * Creates a new {@link TalonFX}
   * @param deviceID The CAN ID of the {@link TalonFX} 
   * @param config A method that takes in a {@link TalonFX}, and configures it as 
   * the caller sees fit. Most of the config should also apply during simulation
   * @param model A {@link DCMotor} that represents the motor that this TalonFX is controlling. Do not apply a
   * gear ratio to this object, as gear ratios are dealt with separately. Also, do not set the motor count to more than one.
   */
  public TalonFXIO(int deviceID, Consumer<TalonFX> config, DCMotor model) {
    this(deviceID, new CANBus(), config, model);
  }

  /**
   * Creates a new {@link TalonFX}
   * @param deviceID The CAN ID of the {@link TalonFX} 
   * @param canbus The canbus this motor is connected to
   * @param config A {@link TalonFXConfiguration} that is applied to this {@link TalonFX}.
   * Configuration is attempted 5 times before giving up. Most of the config should also
   * apply during simulation
   * @param model A {@link DCMotor} that represents the motor that this TalonFX is controlling. Do not apply a
   * gear ratio to this object, as gear ratios are dealt with separately. Also, do not set the motor count to more than one.
   */
  public TalonFXIO(int deviceID, CANBus canbus, TalonFXConfiguration config, DCMotor model) {
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

  /**
   * Creates a new {@link TalonFX}
   * @param deviceID The CAN ID of the {@link TalonFX} 
   * @param config A {@link TalonFXConfiguration} that is applied to this {@link TalonFX}.
   * Configuration is attempted 5 times before giving up. Most of the config should also
   * apply during simulation
   * @param model A {@link DCMotor} that represents the motor that this TalonFX is controlling. Do not apply a
   * gear ratio to this object, as gear ratios are dealt with separately. Also, do not set the motor count to more than one.
   */
  public TalonFXIO(int deviceID, TalonFXConfiguration config, DCMotor model) {
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
    TalonFXSimState simState = motor.getSimState();
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
