package frc.robot.hardware.mechanisms.pivots;

import frc.robot.hardware.mechanisms.Mechanism;
import frc.robot.hardware.mechanisms.pivots.PivotStates.PivotState;
import frc.robot.hardware.mechanisms.pivots.PivotStates.PivotTarget;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIO.MotorIOInputs;
import frc.robot.utilities.FeedbackController;

public class Pivot extends Mechanism<PivotState, PivotTarget> {
  protected MotorIO motor;
  protected double conversionFactor;
  protected double kG;
  protected double kS;
  protected double kV;
  protected double kA;
  protected FeedbackController controller;

  public Pivot(
      MotorIO motor,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      FeedbackController controller,
      int simsPerLoop) {
    super(simsPerLoop);
    this.motor = motor;
    this.conversionFactor = conversionFactor;
    this.kG = kG;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.controller = controller;
  }

  public static Pivot fromIdealValues(
      MotorIO motor,
      double gearReduction,
      double massMomentOfInertia,
      double pivotMass,
      double centerOfMassLength,
      FeedbackController feedbackController,
      int simsPerLoop) {
    return new Pivot(
        motor,
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
  public void log(String name) {
    motor.log(name, "Motor");
  }

  public MotorIO getPivotMotor() {
    return motor;
  }

  @Override
  public PivotState getState() {
    return new PivotState(
        motor.getPosition() / conversionFactor, motor.getVelocity() / conversionFactor);
  }

  @Override
  protected void updateSim(double dt) {
    MotorIOInputs inputs = new MotorIOInputs();
    double acceleration =
        (motor.getVoltage()
                - kG * Math.cos(inputs.position / conversionFactor)
                - kS * Math.signum(inputs.velocity)
                - kV * inputs.velocity / conversionFactor)
            / kA;
    acceleration *= conversionFactor; // now in motor units
    double velocity = inputs.velocity + acceleration * dt;
    double position = inputs.position + velocity * dt;
    motor.updateSim(position, acceleration);
  }

  @Override
  protected void moveToTarget() {
    PivotState current = getState();
    PivotTarget target = getTarget().get();
    double feedback = controller.calculate(current.position(), target.position());
    double gravityVolts = Math.cos(current.position()) * kG;
    double staticVolts = Math.signum(current.velocity()) * kS;
    double kineticVolts = controller.getSetpoint().derivative() * kV;
    double feedforward = gravityVolts + staticVolts + kineticVolts;
    motor.setVoltage(feedback + feedforward);
  }
}
