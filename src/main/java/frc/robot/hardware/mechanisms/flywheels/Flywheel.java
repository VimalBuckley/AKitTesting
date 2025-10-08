package frc.robot.hardware.mechanisms.flywheels;

import frc.robot.hardware.mechanisms.Mechanism;
import frc.robot.hardware.mechanisms.flywheels.FlywheelStates.FlywheelState;
import frc.robot.hardware.mechanisms.flywheels.FlywheelStates.FlywheelTarget;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;

public class Flywheel extends Mechanism<FlywheelState, FlywheelTarget> {
  protected MotorIO motor;
  protected double conversionFactor;
  protected double kS;
  protected double kV;
  protected double kA;
  protected FeedbackController controller;

  public Flywheel(
      MotorIO motor,
      double conversionFactor,
      double kS,
      double kV,
      double kA,
      FeedbackController controller,
      int simsPerLoop) {
    super(simsPerLoop);
    this.motor = motor;
    this.conversionFactor = conversionFactor;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.controller = controller;
  }

  public static Flywheel fromIdealValues(
      MotorIO motor,
      double gearReduction,
      double massMomentOfInertia,
      FeedbackController controller,
      int simsPerLoop) {
    return new Flywheel(
        motor,
        gearReduction,
        0,
        1 / motor.getModel().KvRadPerSecPerVolt * gearReduction,
        massMomentOfInertia
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        controller,
        simsPerLoop);
  }

  @Override
  public void log(String name) {
    motor.log(name, "Motor");
  }

  @Override
  public FlywheelState getState() {
    return new FlywheelState(
        motor.getPosition() / conversionFactor, motor.getVelocity() / conversionFactor);
  }

  @Override
  protected void updateSim(double dt) {
    MotorIOInputs inputs = new MotorIOInputs();
    motor.updateInputs(inputs);
    // acceleration in mechanism units
    double acceleration =
        (inputs.statorVoltage
                - kV * inputs.velocity / conversionFactor
                - kS * Math.signum(inputs.velocity))
            / kA;
    acceleration *= conversionFactor; // now in motor units
    double velocity = inputs.velocity + acceleration * dt;
    double position = inputs.position + velocity * dt;
    motor.updateSim(position, velocity);
  }

  @Override
  protected void moveToTarget() {
    FlywheelState current = getState();
    FlywheelTarget target = getTarget().get();
    double feedback = controller.calculate(current.velocity(), target.velocity());
    double staticVolts = Math.signum(current.velocity()) * kS;
    double kineticVolts = controller.getSetpoint().target() * kV;
    double accelVolts = controller.getSetpoint().derivative() * kA;
    double feedforward = staticVolts + kineticVolts + accelVolts;
    motor.setVoltage(feedback + feedforward);
  }

  public MotorIO getSpinMotor() {
    return motor;
  }
}
