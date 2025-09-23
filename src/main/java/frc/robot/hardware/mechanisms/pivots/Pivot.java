package frc.robot.hardware.mechanisms.pivots;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.Loggable;

public class Pivot extends SubsystemBase implements Loggable {
  protected MotorIO motor;
  protected double target;
  protected boolean enabled;
  protected double conversionFactor;
  protected double kG;
  protected double kS;
  protected double kV;
  protected double kA;
  protected int simsPerLoop;
  protected FeedbackController feedbackController;

  public Pivot(
      MotorIO motor,
      int simsPerLoop,
      double gearReduction,
      double massMomentOfInertia,
      double pivotMass,
      double centerOfMassLength,
      FeedbackController feedbackController) {
    this(
        motor,
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

  public Pivot(
      MotorIO motor,
      int simsPerLoop,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    this.motor = motor;
    this.simsPerLoop = simsPerLoop;
    this.conversionFactor = conversionFactor;
    this.kG = kG;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.feedbackController = feedbackController;
  }

  public void setTargetAngle(double angle) {
    setTargetAngle(angle, true);
  }

  public void setTargetAngle(double angle, boolean enable) {
    target = angle;
    enabled = enable;
  }

  public void enableMechanism() {
    enabled = true;
  }

  public void disableMechanism() {
    enabled = false;
  }

  public double getPosition() {
    return motor.getPosition() / conversionFactor;
  }

  public double getVelocity() {
    return motor.getVelocity() / conversionFactor;
  }

  public MotorIO getPivotMotor() {
    return motor;
  }

  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }
    double feedback = feedbackController.calculate(getPosition(), target);
    double gravityVolts = Math.cos(getPosition()) * kG;
    double staticVolts = Math.signum(getVelocity()) * kS;
    double kineticVolts = feedbackController.getSetpoint().getSecond();
    double feedforward = gravityVolts + staticVolts + kineticVolts;
    motor.setVoltage(feedback + feedforward);
  }

  @Override
  public void simulationPeriodic() {
    MotorIOInputs inputs = new MotorIOInputs();
    for (int i = 0; i < simsPerLoop; i++) {
      motor.updateInputs(inputs);
      double acceleration =
          (motor.getVoltage()
                  - kG * Math.cos(inputs.position / conversionFactor)
                  - kS * Math.signum(inputs.velocity)
                  - kV * inputs.velocity / conversionFactor)
              / kA;
      double velocity = inputs.velocity / conversionFactor + 0.02 / simsPerLoop * acceleration;
      velocity *= conversionFactor;
      motor.updateSim(velocity, 0.02 / simsPerLoop, inputs);
    }
  }

  @Override
  public void log(String name) {
    motor.log(name, "Motor");
  }
}
