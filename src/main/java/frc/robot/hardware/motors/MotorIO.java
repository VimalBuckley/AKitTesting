package frc.robot.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** A class used to interface with motor controllers */
public abstract class MotorIO implements Loggable, IOLayer {
  private DCMotor model;
  private MotorIOInputsAutoLogged inputs;
  private MotorIOConfig config;

  @AutoLog
  public static class MotorIOInputs {
    public double statorVoltage;
    public double supplyVoltage;
    public double position;
    public double velocity;
    public double statorCurrent;
    public double supplyCurrent;
    public double temperature;
  }

  /**
   * Creates a new {@link MotorIO}
   *
   * @param model A {@link DCMotor} that represents the motor that this motor controller is
   *     controlling. Do not apply a gear ratio to this object, as gear ratios are dealt with
   *     separately. Also, do not set the motor count to more than one.
   */
  public MotorIO(DCMotor model, MotorIOConfig config) {
    this.model = model;
    this.config = new MotorIOConfig();
    inputs = new MotorIOInputsAutoLogged();
    applyConfig(config);
    updateInputs();
    Robot.ios.add(this);
  }

  /**
   * Tells the motor to apply the given voltage
   *
   * @param volts The stator voltage to apply to the motor
   */
  public abstract void setVoltage(double volts);

  /**
   * Resets the position of the motor's encoder
   *
   * @param newValue The new position of the motor's encoder, in radians of the motor shaft
   */
  public abstract void setPosition(double newValue);

  /**
   * Updates the given inputs to contain the motor's current state. This should really only be used
   * for sim purposes to allow for multiple sim runs per loop, as using it breaks the IO layer
   * contract, and thus breaks replaying for the surrounding code.
   *
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(MotorIOInputs inputs);

  /**
   * Steps the motor's simulation forward dt seconds. The motor will use the provided inputs
   *
   * @param velocity The velocity of the motor at the end of the timestep, in radians/second at the
   *     motor shaft
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for the simulation
   */
  public void updateSim(double position, double velocity) {
    throw new UnsupportedOperationException("Real motors do not support simulation");
  }

  public void applyConfig(MotorIOConfig config) {
    applyConfigToHardware(config);
    config.copyTo(this.config);
  }

  protected abstract void applyConfigToHardware(MotorIOConfig config);

  /**
   * @return a model of the motor being controlled
   */
  public DCMotor getModel() {
    return model;
  }

  /**
   * @return the "current" position of the motor shaft in radians
   */
  public double getPosition() {
    return inputs.position;
  }

  /**
   * @return the "current" velocity of the motor shaft in radians/second
   */
  public double getVelocity() {
    return inputs.velocity;
  }

  /**
   * @return the "current" voltage being applied to the motor shaft
   */
  public double getVoltage() {
    return inputs.statorVoltage;
  }

  public void refreshConfig(MotorIOConfig config) {
    this.config.copyTo(config);
  }

  @Override
  public void updateInputs() {
    updateInputs(inputs);

    if (RobotBase.isSimulation()) {
      Robot.supplyCurrents.add(inputs.supplyCurrent);
    }
  }

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }

  public static MotorIO makeTalonFX(
      int deviceID, CANBus canbus, DCMotor model, MotorIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(model, config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(model, config);
    }
    return new MotorIO(model, config) {
      TalonFX motor = new TalonFX(deviceID, canbus);

      @Override
      public void setVoltage(double volts) {
        motor.setVoltage(volts);
      }

      @Override
      public void setPosition(double newValue) {
        motor.setPosition(Units.radiansToRotations(newValue));
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

      @Override
      public void applyConfigToHardware(MotorIOConfig config) {
        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        fxConfig.CurrentLimits.StatorCurrentLimit = config.statorLimit;
        fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        fxConfig.CurrentLimits.SupplyCurrentLimit = config.supplyLimit;
        fxConfig.MotorOutput.Inverted = config.inversion;
        fxConfig.MotorOutput.NeutralMode = config.neutralMode;
        fxConfig.Voltage.PeakForwardVoltage = config.maxPositiveVoltage;
        fxConfig.Voltage.PeakReverseVoltage = config.maxNegativeVoltage;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
          status = motor.getConfigurator().apply(fxConfig);
        }
      }
    };
  }

  public static MotorIO makeTalonFXS(
      int deviceID, CANBus canbus, DCMotor model, MotorIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(model, config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(model, config);
    }
    return new MotorIO(model, config) {
      TalonFXS motor = new TalonFXS(deviceID, canbus);

      @Override
      public void setVoltage(double volts) {
        motor.setVoltage(volts);
      }

      @Override
      public void setPosition(double newValue) {
        motor.setPosition(Units.radiansToRotations(newValue));
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

      @Override
      public void applyConfigToHardware(MotorIOConfig config) {
        TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();
        fxsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        fxsConfig.CurrentLimits.StatorCurrentLimit = config.statorLimit;
        fxsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        fxsConfig.CurrentLimits.SupplyCurrentLimit = config.supplyLimit;
        fxsConfig.MotorOutput.Inverted = config.inversion;
        fxsConfig.MotorOutput.NeutralMode = config.neutralMode;
        fxsConfig.Voltage.PeakForwardVoltage = config.maxPositiveVoltage;
        fxsConfig.Voltage.PeakReverseVoltage = config.maxNegativeVoltage;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
          status = motor.getConfigurator().apply(fxsConfig);
        }
      }
    };
  }

  public static MotorIO makeSparkMax(int deviceID, DCMotor model, MotorIOConfig config) {
    // Disable supply limit, since SparkMax's don't support it
    config.supplyLimit = config.statorLimit;
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(model, config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(model, config);
    }
    return new MotorIO(model, config) {
      SparkMax motor = new SparkMax(deviceID, MotorType.kBrushless);

      @Override
      public void setVoltage(double volts) {
        motor.setVoltage(volts);
      }

      @Override
      public void setPosition(double newValue) {
        motor.getEncoder().setPosition(newValue);
      }

      @Override
      public void updateInputs(MotorIOInputs inputs) {
        inputs.statorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyVoltage = motor.getBusVoltage();
        inputs.position = Units.rotationsToRadians(motor.getEncoder().getPosition());
        inputs.velocity =
            Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
        inputs.statorCurrent = motor.getOutputCurrent();
        inputs.supplyCurrent = motor.getAppliedOutput() * inputs.statorCurrent;
        inputs.temperature = motor.getMotorTemperature();
      }

      @Override
      protected void applyConfigToHardware(MotorIOConfig config) {
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.smartCurrentLimit(config.statorLimit);
        sparkConfig.inverted(config.inversion == InvertedValue.Clockwise_Positive);
        sparkConfig.idleMode(
            config.neutralMode == NeutralModeValue.Brake ? IdleMode.kBrake : IdleMode.kCoast);
        sparkConfig.closedLoop.outputRange(
            config.maxNegativeVoltage / 12, config.maxPositiveVoltage / 12);
        REVLibError status = REVLibError.kUnknown;
        for (int i = 0; i < 5 && status != REVLibError.kOk; i++) {
          motor.configure(
              sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
      }
    };
  }

  public static MotorIO makeSim(DCMotor model, MotorIOConfig config) {
    MotorIOConfig hardwareConfig = new MotorIOConfig();
    return new MotorIO(model, config) {
      double position = 0;
      double velocity = 0;
      double targetVoltage = 0;

      @Override
      public void setVoltage(double volts) {
        targetVoltage = volts;
      }

      @Override
      public void setPosition(double newValue) {
        position = newValue;
      }

      @Override
      public void updateInputs(MotorIOInputs inputs) {
        inputs.position = position;
        inputs.velocity = velocity;
        inputs.supplyVoltage = RobotController.getBatteryVoltage();
        inputs.temperature = 0;
        double kV = 1 / model.KvRadPerSecPerVolt;
        double maxVoltage1 = hardwareConfig.statorLimit * model.rOhms + kV * inputs.velocity;
        double minVoltage1 = -hardwareConfig.statorLimit * model.rOhms + kV * inputs.velocity;
        double maxVoltage2 =
            inputs.supplyVoltage * hardwareConfig.supplyLimit / hardwareConfig.statorLimit;
        double minVoltage2 = -maxVoltage2;
        inputs.statorVoltage =
            MathUtil.clamp(
                targetVoltage,
                Math.max(minVoltage1, Math.max(minVoltage2, hardwareConfig.maxNegativeVoltage)),
                Math.min(maxVoltage1, Math.min(maxVoltage2, hardwareConfig.maxPositiveVoltage)));
        inputs.statorCurrent = (inputs.statorVoltage - kV * inputs.velocity) / model.rOhms;
        inputs.supplyCurrent = inputs.statorVoltage / inputs.supplyVoltage * inputs.statorCurrent;
      }

      @Override
      protected void applyConfigToHardware(MotorIOConfig config) {
        config.copyTo(hardwareConfig);
      }

      @Override
      public void updateSim(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
      }
    };
  }

  public static MotorIO makeEmpty(DCMotor model, MotorIOConfig config) {
    return new MotorIO(model, config) {
      @Override
      public void setVoltage(double volts) {}

      @Override
      public void setPosition(double newValue) {}

      @Override
      public void updateInputs(MotorIOInputs inputs) {}

      @Override
      protected void applyConfigToHardware(MotorIOConfig config) {}

      @Override
      public void updateSim(double position, double velocity) {}
    };
  }
}
