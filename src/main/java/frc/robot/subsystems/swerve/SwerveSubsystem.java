package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.gyros.NavXIO;
import frc.robot.hardware.mechanisms.swerve.SwerveDrive;
import frc.robot.hardware.mechanisms.swerve.SwerveModule;
import frc.robot.hardware.tagCameras.TagCameraIO;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive drive;

    public SwerveSubsystem() {
        drive = new SwerveDrive(
            new SwerveModule[] {
                SwerveConstants.FRONT_LEFT_MODULE,
                SwerveConstants.FRONT_RIGHT_MODULE,
                SwerveConstants.BACK_LEFT_MODULE,
                SwerveConstants.BACK_RIGHT_MODULE
            }, 
            new NavXIO("Swerve/NavX", NavXComType.kMXP_SPI), 
            new TagCameraIO[] {}, 
            new ChassisSpeeds(5, 5, 6), 
            5.4, 
            VecBuilder.fill(0.1, 0.1, 0.1), 
            VecBuilder.fill(5, 5, 5),
            new PIDConstants(5),
            new PIDConstants(1),
            this
        );
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.out.println(e.getMessage());
            config = null; // TODO fix
        }
        AutoBuilder.configure(
            drive::getPose, 
            drive::setPose, 
            drive::getSpeeds, 
            drive::setRobotRelativeTargetSpeeds, 
            new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(1)), 
            config, 
            () -> {
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                return alliance == Alliance.Red;
            }, 
            this
        );
    }

    public Command angleCentric(XboxController xbox) {
        return Commands.none();
    }

    public Command robotCentric(XboxController xbox) {
        return Commands.none();
    }
}
