package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.motor.MotorIO;

public class FlywheelMechanism extends SubsystemBase implements Mechanism {
  private MotorIO spinMotor;
  private double kS;
  private double gearReduction;
  private double massMomentOfInertia;
  private PIDController pid;

  private double target;

  public FlywheelMechanism(MotorIO spinMotor, double gearReduction, double massMomentOfInertia) {}

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
    double feedback = pid.calculate(spinMotor.getVelocity() / gearReduction, target);
    double feedforwardTorque = Math.signum(feedback) * kS / gearReduction;
    double feedforward =
        spinMotor.getModel().getCurrent(feedforwardTorque, spinMotor.getVelocity());
    spinMotor.setCurrent(feedback + feedforward);
  }

  @Override
  public void updateSim(double dt) {
    double acceleration = (spinMotor.getTorque() * gearReduction - kS) / massMomentOfInertia;
    spinMotor.updateSim(acceleration, dt);
  }
}
