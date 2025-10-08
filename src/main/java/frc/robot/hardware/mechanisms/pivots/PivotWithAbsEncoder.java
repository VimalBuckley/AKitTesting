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
      FeedbackController feedbackController,
      int simsPerLoop) {
    super(motor, conversionFactor, kG, kS, kV, kA, feedbackController, simsPerLoop);
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
        feedbackController,
        simsPerLoop);
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
