package frc.robot.hardware.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class DCMotorModel extends DCMotor {

  public DCMotorModel(
      double nominalVoltageVolts,
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRadPerSec,
      int numMotors) {
    super(
        nominalVoltageVolts,
        stallTorqueNewtonMeters,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec,
        numMotors);
  }

  public DCMotorModel(DCMotor motor) {
    super(
        motor.nominalVoltageVolts,
        motor.stallTorqueNewtonMeters,
        motor.stallCurrentAmps,
        motor.freeCurrentAmps,
        motor.freeSpeedRadPerSec,
        1);
  }

  public double getCurrent(
      Optional<Integer> statorLimitAmps,
      Optional<Integer> supplyLimitAmps,
      double speedRadiansPerSecond,
      double voltageInputVolts) {
    double speedPercent = speedRadiansPerSecond / freeSpeedRadPerSec;
    double statorCurrent =
        stallCurrentAmps * (voltageInputVolts / 12 - speedPercent) + freeCurrentAmps * speedPercent;
    if (supplyLimitAmps.isPresent()) {
      double maxCurrent =
          0.5
              * (-(stallCurrentAmps - freeCurrentAmps) * speedPercent
                  + Math.sqrt(
                      Math.pow((stallCurrentAmps - freeCurrentAmps) * speedPercent, 2)
                          + 4 * stallCurrentAmps * supplyLimitAmps.get()));
      statorCurrent = MathUtil.clamp(statorCurrent, -maxCurrent, maxCurrent);
    }
    if (statorLimitAmps.isPresent()) {
      statorCurrent = MathUtil.clamp(statorCurrent, -statorLimitAmps.get(), statorLimitAmps.get());
    }
    return statorCurrent;
  }

  public double getTorque(
      Optional<Integer> statorLimitAmps,
      Optional<Integer> supplyLimitAmps,
      double speedRadiansPerSec,
      double voltageInputVolts) {
    double statorCurrent =
        getCurrent(statorLimitAmps, supplyLimitAmps, speedRadiansPerSec, voltageInputVolts);
    return getTorque(statorCurrent, speedRadiansPerSec);
  }

  public double getTorque(double currentAmps, double speedRadiansPerSec) {
    return Math.signum(currentAmps)
        * getTorque(
            Math.abs(currentAmps)
                - freeCurrentAmps * Math.abs(speedRadiansPerSec) / freeSpeedRadPerSec);
  }

  public double getCurrent(double torqueNm, double speedRadiansPerSec) {
    return torqueNm / KtNMPerAmp + freeCurrentAmps * speedRadiansPerSec / freeSpeedRadPerSec;
  }

  public double getVoltage(
      double currentAmps, double speedRadiansPerSec, double voltageInputVolts) {
    return (voltageInputVolts - KvRadPerSecPerVolt * speedRadiansPerSec) / rOhms;
  }

  public double getCurrentLimitedVoltage(
      Optional<Integer> statorLimitAmps,
      Optional<Integer> supplyLimitAmps,
      double speedRadiansPerSecond,
      double volts) {
    if (supplyLimitAmps.isPresent()) {
      double speedPercentCurrent =
          (speedRadiansPerSecond / freeSpeedRadPerSec) * (stallCurrentAmps - freeCurrentAmps);
      double supplyLimitedVolts =
          0.5
              * 12
              * (1 / stallCurrentAmps)
              * (speedPercentCurrent
                  + Math.sqrt(
                      Math.pow(speedPercentCurrent, 2)
                          + 4 * stallCurrentAmps * supplyLimitAmps.get()));
      volts = MathUtil.clamp(volts, -supplyLimitedVolts, supplyLimitedVolts);
    }
    if (statorLimitAmps.isPresent()) {
      double speedPercent = speedRadiansPerSecond / freeSpeedRadPerSec;
      double statorLimitedVolts =
          12
              * (speedPercent
                  + 1
                      / stallCurrentAmps
                      * (statorLimitAmps.get() - freeCurrentAmps * speedPercent));
      volts = MathUtil.clamp(volts, -statorLimitedVolts, statorLimitedVolts);
    }
    return volts;
  }
}
