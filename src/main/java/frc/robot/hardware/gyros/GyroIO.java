package frc.robot.hardware.gyros;

import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A class used to interface with gyros
 */
public abstract class GyroIO implements IOLayer, Loggable {
  private GyroIOInputsAutoLogged inputs;

  /**
   * Creates a new {@link GyroIO}
   */
  public GyroIO() {
    inputs = new GyroIOInputsAutoLogged();
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

  @Override
  public void updateInputs() {
    updateInputs(inputs);
  }

  /**
   * Updates the given inputs to contain the gyros's current state.
   * This should really only be used for sim purposes to allow for multiple
   * sim runs per loop, as using it breaks the IO layer contract, and thus breaks
   * replaying for the surrounding code.
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(GyroIOInputs inputs);

  /**
   * Steps the gyro's simulation forward dt seconds. The gyro will use the provided inputs
   * @param velocity The velocity of the gyro at the end of the timestep, in radians/second
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for the simulation
   */
  public abstract void updateSim(double velocity, double dt, GyroIOInputs inputs);

  @Override
  public void log(String name) {
    Logger.processInputs(name, inputs);
  }
}
