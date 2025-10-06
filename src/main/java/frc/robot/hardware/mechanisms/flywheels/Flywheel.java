package frc.robot.hardware.mechanisms.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.Loggable;

public class Flywheel extends SubsystemBase implements Loggable {
  protected MotorIO motor;
  protected double target;
  protected boolean enabled;
  protected double conversionFactor;
  protected double kS;
  protected double kV;
  protected double kA;
  protected int simsPerLoop;
  protected FeedbackController feedbackController;

  public Flywheel(
      MotorIO motor,
      int simsPerLoop,
      double conversionFactor,
      double kS,
      double kV,
      double kA,
      FeedbackController feedbackController) {
    this.motor = motor;
    this.simsPerLoop = simsPerLoop;
    this.conversionFactor = conversionFactor;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.feedbackController = feedbackController;
  }

  public static Flywheel fromIdealValues(
      MotorIO motor,
      int simsPerLoop,
      double gearReduction,
      double massMomentOfInertia,
      FeedbackController feedbackController) {
    return new Flywheel(
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
    MotorIOInputs inputs = new MotorIOInputs();
    for (int i = 0; i < simsPerLoop; i++) {
      motor.updateInputs(inputs);
      // acceleration in mechanism units
      double acceleration =
          (inputs.statorVoltage
                  - kV * inputs.velocity / conversionFactor
                  - kS * Math.signum(inputs.velocity))
              / kA;
      acceleration *= conversionFactor; // now in motor units
      double velocity = inputs.velocity + acceleration * 0.02 / simsPerLoop;
      double position = inputs.position + velocity * 0.02 / simsPerLoop;
      motor.updateSim(position, velocity);
    }
  }

  @Override
  public void log(String name) {
    motor.log(name, "Motor");
  }
}
