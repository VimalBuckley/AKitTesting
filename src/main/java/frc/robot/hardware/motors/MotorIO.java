package frc.robot.hardware.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.hardware.IOLayer;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A class used to interface with motor controllers
 */
public abstract class MotorIO implements Loggable, IOLayer {
  private DCMotor model;
  private MotorIOInputsAutoLogged inputs;

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
   * @param model A {@link DCMotor} that represents the motor that this motor controller is controlling. Do not apply a
   * gear ratio to this object, as gear ratios are dealt with separately. Also, do not set the motor count to more than one.
   */
  public MotorIO(DCMotor model) {
    this.model = model;
    inputs = new MotorIOInputsAutoLogged();
    Robot.ios.add(this);
  }

  /**
   * Tells the motor to apply the given voltage
   * @param volts The stator voltage to apply to the motor
   */
  public abstract void setVoltage(double volts);

  /**
   * Resets the position of the motor's encoder
   * @param newValue The new position of the motor's encoder, in radians of the motor shaft
   */
  public abstract void setPosition(double newValue);

  /**
   * Updates the given inputs to contain the motor's current state.
   * This should really only be used for sim purposes to allow for multiple
   * sim runs per loop, as using it breaks the IO layer contract, and thus breaks
   * replaying for the surrounding code.
   * @param inputs The inputs to be updated
   */
  public abstract void updateInputs(MotorIOInputs inputs);

  /**
   * Steps the motor's simulation forward dt seconds. The motor will use the provided inputs
   * @param velocity The velocity of the motor at the end of the timestep, in radians/second at the motor shaft
   * @param dt The magnitude of the timestep
   * @param inputs The inputs to use for the simulation
   */
  public abstract void updateSim(double velocity, double dt, MotorIOInputs inputs);

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
}
