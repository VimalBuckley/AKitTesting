package frc.robot.hardware.gyros;

public class GyroIOConfig {
  public double zeroReading = 0;
  public boolean inverted = false;

  public GyroIOConfig withZeroReading(double zeroReading) {
    this.zeroReading = zeroReading;
    return this;
  }

  public GyroIOConfig withInversion(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public void copyTo(GyroIOConfig config) {
    config.zeroReading = zeroReading;
    config.inverted = inverted;
  }
}
