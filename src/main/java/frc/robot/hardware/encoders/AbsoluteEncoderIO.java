package frc.robot.hardware.encoders;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** A class used to interface with absolute encoders */
public abstract class AbsoluteEncoderIO implements IOLayer, Loggable {
  private AbsoluteEncoderIOInputsAutoLogged inputs;
  private AbsoluteEncoderIOConfig config;

  /** Creates a new {@link AbsoluteEncoderIO} */
  public AbsoluteEncoderIO(AbsoluteEncoderIOConfig config) {
    inputs = new AbsoluteEncoderIOInputsAutoLogged();
    applyConfig(config);
    Robot.ios.add(this);
  }

  @AutoLog
  public static class AbsoluteEncoderIOInputs {
    public double position;
  }

  /**
   * @return The "current" position of the absolute encoder, in radians
   */
  public double getPosition() {
    return inputs.position;
  }

  public void applyConfig(AbsoluteEncoderIOConfig config) {
    applyConfigToHardware(config);
    config.copyTo(this.config);
  }

  public void refreshConfig(AbsoluteEncoderIOConfig config) {
    this.config.copyTo(config);
  }

  @Override
  public void updateInputs() {
    updateInputs(inputs);
  }

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }

  /**
   * Updates the given inputs to contain the encoder's current state. This should really only be
   * used for sim purposes to allow for multiple sim runs per loop, as using it breaks the IO layer
   * contract, thus breaks replaying for the surrounding code
   *
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(AbsoluteEncoderIOInputs inputs);

  /**
   * Steps the encoder's simulation forward dt seconds. The encoder will use the provided inputs
   *
   * @param velocity The velocity of the encoder at the end of the timestep, in radians/second
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for simulation
   */
  public void updateSim(double position) {
    throw new UnsupportedOperationException("Real encoders do not support simulation");
  }

  protected abstract void applyConfigToHardware(AbsoluteEncoderIOConfig config);

  public static AbsoluteEncoderIO makeAnalogEncoder(int channel, AbsoluteEncoderIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(config);
    }
    return new AbsoluteEncoderIO(config) {
      AnalogEncoder encoder = new AnalogEncoder(channel);

      @Override
      public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.position =
            MathUtil.angleModulus(Units.rotationsToRadians(encoder.get()) - config.zeroReading);
      }

      @Override
      protected void applyConfigToHardware(AbsoluteEncoderIOConfig config) {
        encoder.setInverted(config.inverted);
      }
    };
  }

  public static AbsoluteEncoderIO makeCANcoder(
      int deviceID, CANBus canbus, AbsoluteEncoderIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(config);
    }
    return new AbsoluteEncoderIO(config) {
      CANcoder encoder = new CANcoder(deviceID, canbus);

      @Override
      public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.position =
            MathUtil.angleModulus(
                Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()));
      }

      @Override
      protected void applyConfigToHardware(AbsoluteEncoderIOConfig config) {
        CANcoderConfiguration hardwareConfig = new CANcoderConfiguration();
        hardwareConfig.MagnetSensor.SensorDirection =
            config.inverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        hardwareConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(-config.zeroReading);
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
          status = encoder.getConfigurator().apply(hardwareConfig);
        }
      }
    };
  }

  public static AbsoluteEncoderIO makeSim(AbsoluteEncoderIOConfig config) {
    return new AbsoluteEncoderIO(config) {
      public double position;

      @Override
      public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.position = MathUtil.angleModulus(position);
      }

      @Override
      protected void applyConfigToHardware(AbsoluteEncoderIOConfig config) {}

      @Override
      public void updateSim(double position) {
        this.position = position;
      }
    };
  }

  public static AbsoluteEncoderIO makeEmpty(AbsoluteEncoderIOConfig config) {
    return new AbsoluteEncoderIO(config) {
      @Override
      public void updateInputs(AbsoluteEncoderIOInputs inputs) {}

      @Override
      protected void applyConfigToHardware(AbsoluteEncoderIOConfig config) {}

      @Override
      public void updateSim(double position) {}
    };
  }
}
