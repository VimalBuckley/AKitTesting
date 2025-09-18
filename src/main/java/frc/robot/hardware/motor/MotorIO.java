package frc.robot.hardware.motor;

import frc.robot.hardware.IOLayer;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.system.plant.DCMotor;

public interface MotorIO extends IOLayer {
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

  public void setVoltage(double volts);

  public DCMotor getModel();

  public double getPosition();

  public double getVelocity();

  public double getVoltage();

  public void setPosition(double newValue);

  public void updateSim(double acceleration, double dt);
}
