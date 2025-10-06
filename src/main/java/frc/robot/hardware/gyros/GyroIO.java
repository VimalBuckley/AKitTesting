package frc.robot.hardware.gyros;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** A class used to interface with gyros */
public abstract class GyroIO implements IOLayer, Loggable {
  private GyroIOInputsAutoLogged inputs;
  private GyroIOConfig config;

  /** Creates a new {@link GyroIO} */
  public GyroIO(GyroIOConfig config) {
    inputs = new GyroIOInputsAutoLogged();
    this.config = new GyroIOConfig();
    applyConfig(config);
    updateInputs();
    Robot.ios.add(this);
  }

  @AutoLog
  public static class GyroIOInputs {
    public double angle;
    public double velocity;
    public boolean isConnected;
  }

  /**
   * @return The "current" rotation of the gyro, in radians
   */
  public double getAngle() {
    return inputs.angle;
  }

  /**
   * @return The "current" velocity of the gyro, in radians/second
   */
  public double getVelocity() {
    return inputs.velocity;
  }

  public void applyConfig(GyroIOConfig config) {
    applyConfigToHardware(config);
    config.copyTo(this.config);
  }

  public void refreshConfig(GyroIOConfig config) {
    this.config.copyTo(config);
  }

  @Override
  public void updateInputs() {
    updateInputs(inputs);
  }

  /**
   * Steps the gyro's simulation forward dt seconds. The gyro will use the provided inputs
   *
   * @param velocity The velocity of the gyro at the end of the timestep, in radians/second
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for the simulation
   */
  public void updateSim(double position, double velocity) {
    throw new UnsupportedOperationException("Real encoders do not support simulation");
  }

  /**
   * Updates the given inputs to contain the gyros's current state. This should really only be used
   * for sim purposes to allow for multiple sim runs per loop, as using it breaks the IO layer
   * contract, and thus breaks replaying for the surrounding code.
   *
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(GyroIOInputs inputs);

  public abstract void applyConfigToHardware(GyroIOConfig config);

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }

  public static GyroIO makeNavX(NavXComType connectionType, GyroIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(config);
    }
    return new GyroIO(config) {
      AHRS gyro = new AHRS(connectionType);
      double inverted = 1;

      @Override
      public void updateInputs(GyroIOInputs inputs) {
        inputs.angle = Units.degreesToRadians(-gyro.getAngle() * inverted);
        inputs.velocity = Units.degreesToRadians(-gyro.getRate() * inverted);
        inputs.isConnected = gyro.isConnected();
      }

      @Override
      public void applyConfigToHardware(GyroIOConfig config) {
        gyro.setAngleAdjustment(Units.radiansToDegrees(-config.zeroReading));
        inverted = config.inverted ? -1 : 1;
      }
    };
  }

  public static GyroIO makePigeon2(int deviceID, CANBus canbus, GyroIOConfig config) {
    if (Constants.currentMode == Mode.SIM) {
      return makeSim(config);
    }
    if (Constants.currentMode == Mode.REPLAY) {
      return makeEmpty(config);
    }
    return new GyroIO(config) {
      Pigeon2 gyro = new Pigeon2(deviceID, canbus);
      double inverted = 1;
      double zeroReading = 0;

      @Override
      public void updateInputs(GyroIOInputs inputs) {
        inputs.angle =
            Units.degreesToRadians(gyro.getYaw().getValueAsDouble() * inverted) - zeroReading;
        inputs.velocity =
            Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble() * inverted);
        inputs.isConnected = gyro.isConnected();
      }

      @Override
      public void applyConfigToHardware(GyroIOConfig config) {
        inverted = config.inverted ? -1 : 1;
        zeroReading = config.zeroReading;
      }
    };
  }

  public static GyroIO makeSim(GyroIOConfig config) {
    return new GyroIO(config) {
      double position;
      double velocity;

      @Override
      public void updateInputs(GyroIOInputs inputs) {
        inputs.angle = position;
        inputs.velocity = velocity;
        inputs.isConnected = true;
      }

      @Override
      public void updateSim(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
      }

      @Override
      public void applyConfigToHardware(GyroIOConfig config) {}
    };
  }

  public static GyroIO makeEmpty(GyroIOConfig config) {
    return new GyroIO(config) {
      @Override
      public void updateInputs(GyroIOInputs inputs) {}

      @Override
      public void applyConfigToHardware(GyroIOConfig config) {}

      @Override
      public void updateSim(double position, double velocity) {}
    };
  }
}
