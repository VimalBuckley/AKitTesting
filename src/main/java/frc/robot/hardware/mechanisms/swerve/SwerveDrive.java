package frc.robot.hardware.mechanisms.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.gyros.GyroIO;
import frc.robot.hardware.tagCameras.TagCameraIO;
import frc.robot.hardware.tagCameras.TagCameraIO.PoseEstimate;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements Loggable {
  private SwerveModule[] modules;
  private GyroIO gyro;
  private TagCameraIO[] tagCameras;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private ChassisSpeeds target;
  private ChassisSpeeds maxRobotSpeeds;
  private double maxModuleSpeed;

  public SwerveDrive(
      SwerveModule[] modules,
      GyroIO gyro,
      TagCameraIO[] tagCameras,
      ChassisSpeeds maxRobotSpeeds,
      double maxModuleSpeed,
      Matrix<N3, N1> stateStandardDeviations,
      Matrix<N3, N1> visionStandardDeviations,
      PIDConstants autoTranslationConstants,
      PIDConstants autoRotationConstants,
      Subsystem driveSubsystem) {
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
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println(e.getMessage());
      config = null; // TODO fix
    }
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        this::setRobotRelativeTargetSpeeds,
        new PPHolonomicDriveController(autoTranslationConstants, autoRotationConstants),
        config,
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          return alliance == Alliance.Red;
        },
        driveSubsystem);
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
    Logger.recordOutput("Robot Pose", getPose());
    Logger.recordOutput("Robot Module States", getModuleStates());
    Logger.recordOutput("Robot Target Module States", targetStates);
  }

  @Override
  public void log(String name) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].log(name, "Module " + i);
    }
    gyro.log(name, "Gyro");
    for (int i = 0; i < tagCameras.length; i++) {
      tagCameras[i].log(name, "Tag Camera " + i);
    }
  }
}
