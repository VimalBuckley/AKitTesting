package frc.robot.hardware.mechanisms.pivots;

import frc.robot.hardware.encoders.EncoderIO;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.utilities.FeedbackController;

public class PivotWithAbsEncoderMechanism extends PivotMechanism {
  protected EncoderIO absEncoder;

  public PivotWithAbsEncoderMechanism(
      MotorIO motor,
      EncoderIO absEncoder,
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

  public PivotWithAbsEncoderMechanism(
      MotorIO motor,
      EncoderIO absEncoder,
      int simsPerLoop,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    super(motor, simsPerLoop, conversionFactor, kG, kS, kV, kA, feedbackController);
  }

  @Override
  public double getPosition() {
    return absEncoder.getPosition();
  }
}
