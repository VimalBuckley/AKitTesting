package frc.robot.hardware.mechanisms.elevators;

import frc.robot.hardware.motors.MotorIO;
import frc.robot.utilities.FeedbackController;

public class Elevator {
  protected MotorIO motor;
  protected double target;
  protected boolean enable;
  protected double conversionFactor;
  protected double kG;
  protected double kS;
  protected double kV;
  protected double kA;
  protected int simsPerLoop;
  protected int numMovingStages;
  protected boolean isCascade;
  protected FeedbackController feedbackController;

  public Elevator(
      MotorIO motor,
      int simsPerLoop,
      double conversionFactor,
      double kG,
      double kS,
      double kV,
      double kA,
      int numMovingStages,
      boolean isCascade,
      FeedbackController feedbackController) {
    this.motor = motor;
    this.simsPerLoop = simsPerLoop;
    this.conversionFactor = conversionFactor;
    this.kG = kG;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.numMovingStages = numMovingStages;
    this.isCascade = isCascade;
    this.feedbackController = feedbackController;
  }

  public static Elevator fromIdealValuesCascade(
      MotorIO motor,
      int simsPerLoop,
      double gearReduction,
      double pulleyRadius,
      FeedbackController feedbackController,
      double... movingStageMasses) {
    double mass = 0;
    for (int i = 0; i < movingStageMasses.length; i++) {
      mass += movingStageMasses[i] * (i + 1);
    }
    gearReduction /= movingStageMasses.length;
    return new Elevator(
        motor,
        simsPerLoop,
        gearReduction / pulleyRadius,
        mass
            * 9.81
            * pulleyRadius
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        0,
        1 / motor.getModel().KvRadPerSecPerVolt * gearReduction / pulleyRadius,
        mass
            * pulleyRadius
            * motor.getModel().nominalVoltageVolts
            / gearReduction
            / motor.getModel().stallTorqueNewtonMeters,
        movingStageMasses.length,
        true,
        feedbackController);
  }

  public static interface ElevatorConfiguration {
    
  }
}
