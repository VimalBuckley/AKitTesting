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
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    super(motor, simsPerLoop, conversionFactor, kG, kS, kV, kA, feedbackController);
    this.absEncoder = absEncoder;
  }

  public static PivotWithAbsEncoder fromIdealValues(
      MotorIO motor,
      AbsoluteEncoderIO absEncoder,
      int simsPerLoop,
      double gearReduction,
      double massMomentOfInertia,
      double pivotMass,
      double centerOfMassLength,
      FeedbackController feedbackController) {
    return new PivotWithAbsEncoder(
        motor,
        absEncoder,
        simsPerLoop,
        gearReduction,
        pivotMass
            * 9.81
            * centerOfMassLength
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        0,
        1 / motor.getModel().KvRadPerSecPerVolt * gearReduction,
        massMomentOfInertia
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        feedbackController);
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
      // acceleration in mechanism units
      double acceleration =
          (motor.getVoltage()
                  - kG * Math.cos(encoderInputs.position)
                  - kS * Math.signum(motorInputs.velocity)
                  - kV * motorInputs.velocity / conversionFactor)
              / kA;
      acceleration *= conversionFactor; // now in motor units
      double velocity = motorInputs.velocity + acceleration * 0.02 / simsPerLoop;
      double position = motorInputs.position + velocity * 0.02 / simsPerLoop;
      motor.updateSim(position, velocity);
      velocity /= conversionFactor; // velocity back to mechanism units
      absEncoder.updateSim(encoderInputs.position + velocity * 0.02 / simsPerLoop);
    }
  }

  @Override
  public void log(String name) {
    super.log(name);
    absEncoder.log(name, "Absolute Encoder");
  }
}
