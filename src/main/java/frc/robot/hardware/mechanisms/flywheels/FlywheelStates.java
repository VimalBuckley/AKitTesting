package frc.robot.hardware.mechanisms.flywheels;

public class FlywheelStates {
  public static record FlywheelState(double position, double velocity) {}

  public static record FlywheelTarget(double velocity) {}
}
