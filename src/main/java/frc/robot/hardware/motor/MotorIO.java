package frc.robot.hardware.motor;

import frc.robot.hardware.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO extends IOLayer {
  @AutoLog
  public static class MotorIOInputs {
    public double statorVoltage;
    public double position;
    public double velocity;
    public double statorCurrent;
    public double supplyCurrent;
  }

  public void setVoltage(double volts);

  public void setCurrent(double amps);

  public DCMotorModel getModel();

  public double getPosition();

  public double getVelocity();

  public double getTorque();

  public void setPosition(double newValue);

  public void updateSim(double acceleration, double dt);
}
