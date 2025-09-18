package frc.robot.hardware.gyros;

import frc.robot.hardware.IOLayer;

public interface GyroIO extends IOLayer {
  public double getAngle();

  public double getVelocity();
}
