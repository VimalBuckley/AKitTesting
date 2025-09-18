package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.hardware.motor.MotorIO;
import frc.robot.utilities.FeedbackController;
import org.littletonrobotics.junction.Logger;

public class FlywheelMechanism extends SubsystemBase {
  private MotorIO spinMotor;
  private double kS;
  private double kV;
  private double kA;
  private double gearReduction;
  private FeedbackController feedbackController;
  private double target;

  public FlywheelMechanism(
      String name,
      MotorIO spinMotor,
      double gearReduction,
      double massMomentOfInertia,
      FeedbackController feedbackController) {
    this.spinMotor = spinMotor;
    Robot.ios.put(name + "/Spin Motor", spinMotor);
    this.gearReduction = gearReduction;
    DCMotor model = spinMotor.getModel();
    kS = 0;
    kV = 1 / model.KvRadPerSecPerVolt * gearReduction;
    kA =
        massMomentOfInertia
            * model.nominalVoltageVolts
            / gearReduction
            / model.stallTorqueNewtonMeters;
    this.feedbackController = feedbackController;
  }

  public void setTargetSpeed(double speed) {
    target = speed;
  }

  public double getPosition() {
    return spinMotor.getPosition() / gearReduction;
  }

  public double getVelocity() {
    return spinMotor.getVelocity() / gearReduction;
  }

  @Override
  public void periodic() {
    double feedback = feedbackController.calculate(getVelocity(), target);
    double staticVolts = Math.signum(getVelocity()) * kS;
    double kineticVolts = feedbackController.getSetpoint().getFirst() * kV;
    double accelVolts = feedbackController.getSetpoint().getSecond() * kA;
    double feedforward = staticVolts + kineticVolts + accelVolts;
    Logger.recordOutput("Kinetic Volts", kineticVolts);
    Logger.recordOutput("Accel Volts", accelVolts);
    spinMotor.setVoltage(feedback + feedforward);
  }

  @Override
  public void simulationPeriodic() {
    int simsPerLoop = 1;
    for (int i = 0; i < simsPerLoop; i++) {
      double acceleration =
          (spinMotor.getVoltage() - kV * getVelocity() - kS * Math.signum(getVelocity())) / kA;
      Logger.recordOutput("Acceleration", acceleration);
      spinMotor.updateSim(acceleration, 0.02 / simsPerLoop);
    }
  }
}
