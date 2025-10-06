package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.gyros.GyroIO;
import frc.robot.hardware.gyros.GyroIOConfig;
import frc.robot.hardware.mechanisms.swerve.SwerveDrive;
import frc.robot.hardware.mechanisms.swerve.SwerveModule;
import frc.robot.hardware.tagCameras.TagCameraIO;
import frc.robot.utilities.Loggable;

public class SwerveSubsystem extends SubsystemBase implements Loggable {
  private SwerveDrive drive;

  public SwerveSubsystem() {
    drive =
        new SwerveDrive(
            new SwerveModule[] {
              SwerveConstants.FRONT_LEFT_MODULE,
              SwerveConstants.FRONT_RIGHT_MODULE,
              SwerveConstants.BACK_LEFT_MODULE,
              SwerveConstants.BACK_RIGHT_MODULE
            },
            GyroIO.makeNavX(NavXComType.kMXP_SPI, new GyroIOConfig()),
            new TagCameraIO[] {},
            new ChassisSpeeds(5, 5, 6),
            5.4,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(5, 5, 5),
            new PIDConstants(5),
            new PIDConstants(1),
            this);
  }

  public Command angleCentric(XboxController xbox) {
    return Commands.none();
  }

  public Command robotCentric(XboxController xbox) {
    return run(() -> {});
  }

  @Override
  public void log(String name) {
    drive.log(name, "Swerve Drive");
  }
}
