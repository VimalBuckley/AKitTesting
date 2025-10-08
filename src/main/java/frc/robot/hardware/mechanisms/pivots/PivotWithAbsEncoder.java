package frc.robot.hardware.mechanisms.pivots;

import frc.robot.hardware.encoders.AbsoluteEncoderIO;
import frc.robot.hardware.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOInputs;
import frc.robot.hardware.mechanisms.pivots.PivotStates.PivotState;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;

public class PivotWithAbsEncoder extends Pivot {
  protected AbsoluteEncoderIO absEncoder;

  public PivotWithAbsEncoder(
      MotorIO motor,
      AbsoluteEncoderIO absEncoder,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController controller,
      int simsPerLoop) {
    super(motor, conversionFactor, kG, kS, kV, kA, controller, simsPerLoop);
    this.absEncoder = absEncoder;
  }

  public static PivotWithAbsEncoder fromPivot(Pivot pivot, AbsoluteEncoderIO absEncoder) {
    return new PivotWithAbsEncoder(
        pivot.motor,
        absEncoder,
        pivot.conversionFactor,
        pivot.kG,
        pivot.kS,
        pivot.kV,
        pivot.kA,
        pivot.controller,
        pivot.getSimsPerLoop());
  }

  @Override
  public PivotState getState() {
    return new PivotState(absEncoder.getPosition(), motor.getVelocity() / conversionFactor);
  }

  @Override
  public void updateSim(double dt) {
    MotorIOInputs motorInputs = new MotorIOInputs();
    AbsoluteEncoderIOInputs encoderInputs = new AbsoluteEncoderIOInputs();
    motor.updateInputs(motorInputs);
    absEncoder.updateInputs(encoderInputs);
    // acceleration in mechanism units
    double acceleration =
        (motor.getVoltage()
                - kG * Math.cos(encoderInputs.position)
                - kS * Math.signum(motorInputs.velocity)
                - kV * motorInputs.velocity / conversionFactor)
            / kA;
    acceleration *= conversionFactor; // now in motor units
    double velocity = motorInputs.velocity + acceleration * dt;
    double position = motorInputs.position + velocity * dt;
    motor.updateSim(position, velocity);
    velocity /= conversionFactor; // velocity back to mechanism units
    absEncoder.updateSim(encoderInputs.position + velocity * dt);
  }

  @Override
  public void log(String name) {
    super.log(name);
    absEncoder.log(name, "Absolute Encoder");
  }
}
