package frc.robot.hardware.mechanisms.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.mechanisms.flywheels.FlywheelMechanism;
import frc.robot.hardware.mechanisms.pivots.PivotMechanism;
import frc.robot.hardware.motors.MotorIO;

public class SwerveModule {
  private FlywheelMechanism drive;
  private PivotMechanism angle;
  private Translation2d moduleLocation;
  private double wheelRadius;
  private SwerveModuleState target;

  public SwerveModule(
      FlywheelMechanism drive,
      PivotMechanism angle,
      Translation2d moduleLocation,
      double wheelRadius) {
    this.drive = drive;
    this.angle = angle;
    this.moduleLocation = moduleLocation;
    this.wheelRadius = wheelRadius;
    this.target = getCurrentState();
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
    target.optimize(currentState.angle);
    target.cosineScale(currentState.angle);
    drive.setTargetSpeed(target.speedMetersPerSecond / wheelRadius);
    angle.setTargetAngle(target.angle.getRadians());
  }
}
