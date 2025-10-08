package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import java.util.function.Consumer;

public interface FeedbackController {
  public double calculate(double measurement, double goal);

  public double getGoal();

  public Setpoint getSetpoint();

  public boolean atGoal();

  public void reset(double measurement);

  public static record Setpoint(
    double target,
    double derivative
  ) {}

  public static class PIDFeedback implements FeedbackController {
    private PIDController pid;

    public PIDFeedback(double kP) {
      this(kP, 0, 0);
    }

    public PIDFeedback(double kP, double kI, double kD) {
      this(kP, kI, kD, pid -> {});
    }

    public PIDFeedback(double kP, Consumer<PIDController> config) {
      this(kP, 0, 0, config);
    }

    public PIDFeedback(double kP, double kI, double kD, Consumer<PIDController> config) {
      pid = new PIDController(kP, kI, kD);
      config.accept(pid);
    }

    @Override
    public double calculate(double measurement, double goal) {
      return pid.calculate(measurement, goal);
    }

    @Override
    public double getGoal() {
      return pid.getSetpoint();
    }

    @Override
    public Setpoint getSetpoint() {
      return new Setpoint(getGoal(), 0.);
    }

    @Override
    public boolean atGoal() {
      return pid.atSetpoint();
    }

    @Override
    public void reset(double measurement) {
      pid.calculate(measurement, measurement);
      pid.reset();
    }
  }

  public static class ProfiledFeedback implements FeedbackController {
    private ProfiledPIDController pid;

    public ProfiledFeedback(double kP, Constraints constraints) {
      this(kP, 0, 0, constraints);
    }

    public ProfiledFeedback(double kP, double kI, double kD, Constraints constraints) {
      this(kP, kI, kD, constraints, pid -> {});
    }

    public ProfiledFeedback(
        double kP, Constraints constraints, Consumer<ProfiledPIDController> config) {
      this(kP, 0, 0, constraints, config);
    }

    public ProfiledFeedback(
        double kP,
        double kI,
        double kD,
        Constraints constraints,
        Consumer<ProfiledPIDController> config) {
      pid = new ProfiledPIDController(kP, kI, kD, constraints);
      config.accept(pid);
    }

    @Override
    public double calculate(double measurement, double goal) {
      return pid.calculate(measurement, goal);
    }

    @Override
    public double getGoal() {
      return pid.getGoal().position;
    }

    @Override
    public Setpoint getSetpoint() {
      return new Setpoint(pid.getSetpoint().position, pid.getSetpoint().velocity);
    }

    @Override
    public boolean atGoal() {
      return pid.atGoal();
    }

    @Override
    public void reset(double measurement) {
      pid.calculate(measurement, measurement);
      pid.reset(measurement);
    }
  }
}
