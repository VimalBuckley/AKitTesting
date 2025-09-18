package frc.robot.hardware.mechanisms.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.encoders.EncoderIO;
import frc.robot.hardware.gyros.GyroIO;
import frc.robot.hardware.mechanisms.flywheels.FlywheelMechanism;
import frc.robot.hardware.mechanisms.pivots.PivotMechanism;
import frc.robot.hardware.mechanisms.pivots.PivotWithAbsEncoderMechanism;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.tagCameras.TagCameraIO;
import frc.robot.hardware.tagCameras.TagCameraIO.PoseEstimate;
import frc.robot.utilities.FeedbackController;

public class SwerveMechanism extends SubsystemBase {
  private SwerveModule[] modules;
  private GyroIO gyro;
  private TagCameraIO[] tagCameras;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private ChassisSpeeds target;
  private ChassisSpeeds maxRobotSpeeds;
  private double maxModuleSpeed;

  public SwerveMechanism(
      SwerveModule[] modules,
      GyroIO gyro,
      TagCameraIO[] tagCameras,
      ChassisSpeeds maxRobotSpeeds,
      double maxModuleSpeed,
      Matrix<N3, N1> stateStandardDeviations,
      Matrix<N3, N1> visionStandardDeviations) {
    this.modules = modules;
    this.gyro = gyro;
    this.tagCameras = tagCameras;
    this.maxRobotSpeeds = maxRobotSpeeds;
    this.maxModuleSpeed = maxModuleSpeed;
    target = new ChassisSpeeds();
    kinematics = new SwerveDriveKinematics(getModuleLocations());
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromRadians(gyro.getAngle()),
            getModulePositions(),
            new Pose2d(),
            stateStandardDeviations,
            visionStandardDeviations);
  }

  public static FlywheelMechanism[] makeModuleDrives(
    MotorIO[] driveMotors,
    int simsPerLoop,
    FeedbackController[] controllers,
    double gearReduction,
    double wheelRadius,
    double robotMass
  ) {
    if (!(driveMotors.length == controllers.length)) {
        throw new RuntimeException("There must be an equal amount of Drive Motors and Feedback Controllers");
    }
    FlywheelMechanism[] drives = new FlywheelMechanism[driveMotors.length];
    for (int i = 0; i < drives.length; i++) {
        drives[i] = new FlywheelMechanism(
            driveMotors[i], 
            simsPerLoop, 
            gearReduction, 
            0,
            12
                  / (driveMotors[i].getModel().freeSpeedRadPerSec
                      / gearReduction
                      * wheelRadius
                      * 2
                      * Math.PI),
              12
                  * robotMass
                  * wheelRadius
                  / (drives.length
                      * 2
                      * Math.PI
                      * gearReduction
                      * driveMotors[i].getModel().stallTorqueNewtonMeters), 
            controllers[i]
        );
    }
    return drives;
  }

  public static SwerveModule[] makeModules(
      MotorIO[] driveMotors,
      MotorIO[] angleMotors,
      int driveSimsPerLoop,
      int angleSimsPerLoop,
      EncoderIO[] absoluteEncoders,
      Translation2d[] moduleLocations,
      FeedbackController[] driveControllers,
      FeedbackController[] angleControllers,
      double driveGearReduction,
      double angleGearReduction,
      double wheelRadius,
      double robotMass,
      double angleMassMomentOfInertia) {
    if (!(driveMotors.length == angleMotors.length)
        && (angleMotors.length == absoluteEncoders.length)
        && (absoluteEncoders.length == moduleLocations.length)
        && (moduleLocations.length == driveControllers.length)
        && (driveControllers.length == angleControllers.length)) {
      return null;
    }
    SwerveModule[] modules = new SwerveModule[driveMotors.length];
    for (int i = 0; i < modules.length; i++) {
      FlywheelMechanism drive =
          new FlywheelMechanism(
              driveMotors[i],
              driveSimsPerLoop,
              driveGearReduction,
              0,
              12
                  / (driveMotors[i].getModel().freeSpeedRadPerSec
                      / driveGearReduction
                      * wheelRadius
                      * 2
                      * Math.PI),
              12
                  * robotMass
                  * wheelRadius
                  / (modules.length
                      * 2
                      * Math.PI
                      * driveGearReduction
                      * driveMotors[i].getModel().stallTorqueNewtonMeters),
              driveControllers[i]);
      PivotMechanism angle =
          new PivotWithAbsEncoderMechanism(
              angleMotors[i],
              absoluteEncoders[i],
              angleSimsPerLoop,
              angleGearReduction,
              angleMassMomentOfInertia,
              0,
              0,
              angleControllers[i]);
      modules[i] = new SwerveModule(drive, angle, moduleLocations[i], wheelRadius);
    }
    return modules;
  }

  public void setRobotRelativeTargetSpeeds(ChassisSpeeds speeds) {
    target = speeds;
  }

  public void setFieldRelativeTargetSpeeds(ChassisSpeeds speeds) {
    target = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    estimator.resetPose(pose);
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentPosition();
    }
    return states;
  }

  private Translation2d[] getModuleLocations() {
    Translation2d[] locations = new Translation2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      locations[i] = modules[i].getModuleLocation();
    }
    return locations;
  }

  @Override
  public void periodic() {
    estimator.update(Rotation2d.fromRadians(gyro.getAngle()), getModulePositions());
    for (TagCameraIO camera : tagCameras) {
      PoseEstimate estimate = camera.getPoseEstimate();
      if (estimate.exists()) {
        estimator.addVisionMeasurement(
            estimate.pose(), Timer.getTimestamp() - estimate.latencySeconds());
      }
    }
    ChassisSpeeds target =
        new ChassisSpeeds(
            this.target.vxMetersPerSecond,
            this.target.vyMetersPerSecond,
            this.target.omegaRadiansPerSecond);
    double coefficient = maxRobotSpeeds.vxMetersPerSecond / Math.abs(target.vxMetersPerSecond);
    if (coefficient < 1) {
      target =
          new ChassisSpeeds(
              target.vxMetersPerSecond * coefficient,
              target.vyMetersPerSecond * coefficient,
              target.omegaRadiansPerSecond * coefficient);
    }
    coefficient = maxRobotSpeeds.vyMetersPerSecond / Math.abs(target.vyMetersPerSecond);
    if (coefficient < 1) {
      target =
          new ChassisSpeeds(
              target.vxMetersPerSecond * coefficient,
              target.vyMetersPerSecond * coefficient,
              target.omegaRadiansPerSecond * coefficient);
    }
    coefficient = maxRobotSpeeds.omegaRadiansPerSecond / Math.abs(target.omegaRadiansPerSecond);
    if (coefficient < 1) {
      target =
          new ChassisSpeeds(
              target.vxMetersPerSecond * coefficient,
              target.vyMetersPerSecond * coefficient,
              target.omegaRadiansPerSecond * coefficient);
    }
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(target);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxModuleSpeed);
    target = kinematics.toChassisSpeeds(targetStates);
    target = ChassisSpeeds.discretize(target, 0.02);
    targetStates = kinematics.toSwerveModuleStates(target);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
      modules[i].periodic();
    }
  }
}
