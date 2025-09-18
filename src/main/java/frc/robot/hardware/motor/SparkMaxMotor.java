package frc.robot.hardware.motor;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class SparkMaxMotor implements MotorIO {
  private SparkMax motor;
  private DCMotor model;
  private MotorIOInputsAutoLogged inputs;

  public SparkMaxMotor(int deviceID, Consumer<SparkMax> config, DCMotor model) {
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    config.accept(motor);
    this.model = model;
    inputs = new MotorIOInputsAutoLogged();
  }

  public SparkMaxMotor(int deviceID, SparkMaxConfig config, DCMotor model) {
    this(
        deviceID,
        spark -> {
          REVLibError status = REVLibError.kUnknown;
          for (int i = 0; i < 5 && status != REVLibError.kOk; i++) {
            spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
          }
        },
        model);
  }

  @Override
  public void updateInputs(String name) {
    inputs.statorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyVoltage = motor.getBusVoltage();
    inputs.position = Units.rotationsToRadians(motor.getEncoder().getPosition());
    inputs.velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = motor.getAppliedOutput() * inputs.statorCurrent;
    inputs.temperature = motor.getMotorTemperature();
    Logger.processInputs(name, inputs);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public DCMotor getModel() {
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
  public double getVoltage() {
    return inputs.statorVoltage;
  }

  @Override
  public void setPosition(double newValue) {
    motor.getEncoder().setPosition(newValue);
  }

  @Override
  public void updateSim(double acceleration, double dt) {
    SparkMaxSim sim = new SparkMaxSim(motor, model);
    double vel = inputs.velocity + acceleration * dt;
    sim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(vel), RobotController.getBatteryVoltage(), dt);
  }
}
