package frc.robot.hardware.mechanisms.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.utilities.FeedbackController;

public class FlywheelMechanism extends SubsystemBase {
  protected MotorIO motor;
  protected double target;
  protected boolean enabled;
  protected double conversionFactor;
  protected double kS;
  protected double kV;
  protected double kA;
  protected int simsPerLoop;
  protected FeedbackController feedbackController;

  public FlywheelMechanism(
      MotorIO motor,
      int simsPerLoop,
      double gearReduction,
      double massMomentOfInertia,
      FeedbackController feedbackController) {
    this(
        motor,
        simsPerLoop,
        gearReduction,
        0,
        1 / motor.getModel().KvRadPerSecPerVolt * gearReduction,
        massMomentOfInertia
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        feedbackController);
  }

  public FlywheelMechanism(
      MotorIO motor,
      int simsPerLoop,
      double conversionFactor,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    this.motor = motor;
    Robot.ios.add(motor);
    this.simsPerLoop = simsPerLoop;
    this.conversionFactor = conversionFactor;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.feedbackController = feedbackController;
  }

  public void setTargetSpeed(double speed) {
    setTargetSpeed(speed, true);
  }

  public void setTargetSpeed(double speed, boolean enable) {
    target = speed;
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

  public MotorIO getSpinMotor() {
    return motor;
  }

  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }
    double feedback = feedbackController.calculate(getVelocity(), target);
    double staticVolts = Math.signum(getVelocity()) * kS;
    double kineticVolts = feedbackController.getSetpoint().getFirst() * kV;
    double accelVolts = feedbackController.getSetpoint().getSecond() * kA;
    double feedforward = staticVolts + kineticVolts + accelVolts;
    motor.setVoltage(feedback + feedforward);
  }

  @Override
  public void simulationPeriodic() {
    for (int i = 0; i < simsPerLoop; i++) {
      double acceleration =
          (motor.getVoltage() - kV * getVelocity() - kS * Math.signum(getVelocity())) / kA;
      motor.updateSim(acceleration, 0.02 / simsPerLoop);
    }
  }
}
