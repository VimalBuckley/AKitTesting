package frc.robot.hardware.motors;

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

public class SparkMaxIO extends MotorIO {
  private SparkMax motor;
  private SparkMaxSim sim;

  public SparkMaxIO(int deviceID, Consumer<SparkMax> config, DCMotor model) {
    super(model);
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    config.accept(motor);
    sim = new SparkMaxSim(motor, model);
  }

  public SparkMaxIO(int deviceID, SparkMaxConfig config, DCMotor model) {
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
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double newValue) {
    motor.getEncoder().setPosition(newValue);
  }

  @Override
  public void updateSim(double velocity, double dt, MotorIOInputs inputs) {
    sim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(velocity),
        RobotController.getBatteryVoltage(),
        dt);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.statorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyVoltage = motor.getBusVoltage();
    inputs.position = Units.rotationsToRadians(motor.getEncoder().getPosition());
    inputs.velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = motor.getAppliedOutput() * inputs.statorCurrent;
    inputs.temperature = motor.getMotorTemperature();
  }
}
