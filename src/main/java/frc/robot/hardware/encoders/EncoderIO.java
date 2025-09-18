package frc.robot.hardware.encoders;

import frc.robot.hardware.IOLayer;

public interface EncoderIO extends IOLayer {
  public double getPosition();

  public double setPosition(double newValue);
}
