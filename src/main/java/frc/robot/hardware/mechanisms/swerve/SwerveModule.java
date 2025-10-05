package frc.robot.hardware.mechanisms.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.encoders.AbsoluteEncoderIO;
import frc.robot.hardware.mechanisms.flywheels.Flywheel;
import frc.robot.hardware.mechanisms.pivots.Pivot;
import frc.robot.hardware.mechanisms.pivots.PivotWithAbsEncoder;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.Logger;

public class SwerveModule implements Loggable {
  private Flywheel drive;
  private Pivot angle;
  private Translation2d moduleLocation;
  private double wheelRadius;
  private SwerveModuleState target;
  private SwerveModuleState dynamicTarget;

  public SwerveModule(
      Flywheel drive, Pivot angle, Translation2d moduleLocation, double wheelRadius) {
    this.drive = drive;
    this.angle = angle;
    this.moduleLocation = moduleLocation;
    this.wheelRadius = wheelRadius;
    this.target = getCurrentState();
  }

  public static SwerveModule fromMeasuredValues(
      Translation2d location,
      double wheelRadius,
      MotorIO driveMotor,
      FeedbackController driveFeedback,
      double driveConversionFactor,
      int driveSimsPerLoop,
      double drivekS,
      double drivekV,
      double drivekA,
      MotorIO angleMotor,
      FeedbackController angleFeedback,
      double angleConversionFactor,
      int angleSimsPerLoop,
      double anglekS,
      double anglekV,
      double anglekA,
      AbsoluteEncoderIO absoluteEncoder) {
    return new SwerveModule(
        new Flywheel(
            driveMotor,
            driveSimsPerLoop,
            driveConversionFactor,
            drivekS,
            drivekV,
            drivekA,
            driveFeedback),
        new PivotWithAbsEncoder(
            angleMotor,
            absoluteEncoder,
            angleSimsPerLoop,
            angleConversionFactor,
            0,
            anglekS,
            anglekV,
            anglekA,
            angleFeedback),
        location,
        wheelRadius);
  }

  public static SwerveModule fromIdealValues(
      Translation2d moduleLocation,
      double wheelRadius,
      double robotMass,
      int numModules,
      MotorIO driveMotor,
      int driveSimsPerLoop,
      double driveGearReduction,
      FeedbackController driveFeedback,
      MotorIO angleMotor,
      FeedbackController angleFeedback,
      double angleGearReduction,
      int angleSimsPerLoop,
      double angleMassMomentOfInertia,
      AbsoluteEncoderIO absoluteEncoder) {
    return new SwerveModule(
        new Flywheel(
            driveMotor,
            driveSimsPerLoop,
            driveGearReduction,
            0,
            1 / driveMotor.getModel().KvRadPerSecPerVolt * driveGearReduction,
            driveMotor.getModel().nominalVoltageVolts
                * robotMass
                * wheelRadius
                * wheelRadius
                / (numModules * driveGearReduction * driveMotor.getModel().stallTorqueNewtonMeters),
            driveFeedback),
        PivotWithAbsEncoder.fromIdealValues(
            angleMotor,
            absoluteEncoder,
            angleSimsPerLoop,
            angleGearReduction,
            angleMassMomentOfInertia,
            0,
            0,
            angleFeedback),
        moduleLocation,
        wheelRadius);
  }

  public SwerveModule fromIdealDriveValues(
      Translation2d moduleLocation,
      double wheelRadius,
      double robotMass,
      int numModules,
      MotorIO driveMotor,
      int driveSimsPerLoop,
      double driveGearReduction,
      FeedbackController driveFeedback,
      MotorIO angleMotor,
      FeedbackController angleFeedback,
      double angleConversionFactor,
      int angleSimsPerLoop,
      double anglekS,
      double anglekV,
      double anglekA,
      AbsoluteEncoderIO absoluteEncoder) {
    return new SwerveModule(
        new Flywheel(
            driveMotor,
            driveSimsPerLoop,
            driveGearReduction,
            0,
            1 / driveMotor.getModel().KvRadPerSecPerVolt * driveGearReduction,
            driveMotor.getModel().nominalVoltageVolts
                * robotMass
                * wheelRadius
                * wheelRadius
                / (numModules * driveGearReduction * driveMotor.getModel().stallTorqueNewtonMeters),
            driveFeedback),
        new PivotWithAbsEncoder(
            angleMotor,
            absoluteEncoder,
            angleSimsPerLoop,
            angleConversionFactor,
            0,
            anglekS,
            anglekV,
            anglekA,
            angleFeedback),
        moduleLocation,
        wheelRadius);
  }

  public void setTargetState(SwerveModuleState state) {
    target = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        drive.getVelocity() * wheelRadius,
        Rotation2d.fromRadians(MathUtil.angleModulus(angle.getPosition())));
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(
        drive.getPosition() * wheelRadius,
        Rotation2d.fromRadians(MathUtil.angleModulus(angle.getPosition())));
  }

  public Translation2d getModuleLocation() {
    return moduleLocation;
  }

  public MotorIO getDriveMotor() {
    return drive.getSpinMotor();
  }

  public MotorIO getAngleMotor() {
    return angle.getPivotMotor();
  }

  public void periodic() {
    SwerveModuleState currentState = getCurrentState();
    dynamicTarget = new SwerveModuleState(this.target.speedMetersPerSecond, this.target.angle);
    dynamicTarget.optimize(currentState.angle);
    dynamicTarget.cosineScale(currentState.angle);
    drive.setTargetSpeed(dynamicTarget.speedMetersPerSecond / wheelRadius);
    angle.setTargetAngle(dynamicTarget.angle.getRadians());
  }

  @Override
  public void log(String name) {
    drive.log(name, "Drive");
    angle.log(name, "Angle");
    Logger.recordOutput(name + "/Current State", getCurrentState());
    Logger.recordOutput(name + "/Target State", target);
    Logger.recordOutput(name + "/Dynamic Target State", dynamicTarget);
  }
}
