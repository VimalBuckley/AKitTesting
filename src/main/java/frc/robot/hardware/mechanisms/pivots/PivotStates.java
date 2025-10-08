package frc.robot.hardware.mechanisms.pivots;

public class PivotStates {
  public static record PivotState(double position, double velocity) {}

  public static record PivotTarget(double position) {}
}
