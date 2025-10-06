package frc.robot.hardware.encoders;

public class AbsoluteEncoderIOConfig {
  public double zeroReading = 0;
  public boolean inverted = false;

  public AbsoluteEncoderIOConfig withZeroReading(double zeroReading) {
    this.zeroReading = zeroReading;
    return this;
  }

  public AbsoluteEncoderIOConfig withInversion(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public void copyTo(AbsoluteEncoderIOConfig config) {
    config.zeroReading = zeroReading;
    config.inverted = inverted;
  }
}
