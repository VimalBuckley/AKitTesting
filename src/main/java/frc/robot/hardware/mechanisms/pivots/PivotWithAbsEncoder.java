package frc.robot.hardware.mechanisms.pivots;

import frc.robot.hardware.encoders.AbsoluteEncoderIO;
import frc.robot.hardware.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOInputs;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;

public class PivotWithAbsEncoder extends Pivot {
  protected AbsoluteEncoderIO absEncoder;

  public PivotWithAbsEncoder(
      MotorIO motor,
      AbsoluteEncoderIO absEncoder,
      int simsPerLoop,
      double gearReduction,
      double massMomentOfInertia,
      double pivotMass,
      double centerOfMassLength,
      FeedbackController feedbackController) {
    super(
        motor,
        simsPerLoop,
        gearReduction,
        massMomentOfInertia,
        pivotMass,
        centerOfMassLength,
        feedbackController);
    this.absEncoder = absEncoder;
  }

  public PivotWithAbsEncoder(
      MotorIO motor,
      AbsoluteEncoderIO absEncoder,
      int simsPerLoop,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    super(motor, simsPerLoop, conversionFactor, kG, kS, kV, kA, feedbackController);
    this.absEncoder = absEncoder;
  }

  @Override
  public double getPosition() {
    return absEncoder.getPosition();
  }

  @Override
  public void simulationPeriodic() {
    MotorIOInputs motorInputs = new MotorIOInputs();
    AbsoluteEncoderIOInputs encoderInputs = new AbsoluteEncoderIOInputs();
    for (int i = 0; i < simsPerLoop; i++) {
      motor.updateInputs(motorInputs);
      absEncoder.updateInputs(encoderInputs);
      double acceleration =
          (motor.getVoltage()
                  - kG * Math.cos(encoderInputs.position)
                  - kS * Math.signum(motorInputs.velocity)
                  - kV * motorInputs.velocity / conversionFactor)
              / kA;
      double velocity = motorInputs.velocity / conversionFactor + 0.02 / simsPerLoop * acceleration;
      absEncoder.updateSim(velocity, 0.02 / simsPerLoop, encoderInputs);
      velocity *= conversionFactor;
      motor.updateSim(velocity, 0.02 / simsPerLoop, motorInputs);
    }
  }

  @Override
  public void log(String name) {
    super.log(name);
    absEncoder.log(name, "Absolute Encoder");
  }
}
