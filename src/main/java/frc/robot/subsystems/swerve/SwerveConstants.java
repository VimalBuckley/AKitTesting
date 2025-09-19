package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.WiringConstants.SwerveWiring;
import frc.robot.hardware.encoders.AnalogEncoderIO;
import frc.robot.hardware.mechanisms.swerve.SwerveModule;
import frc.robot.hardware.motors.SparkMaxIO;
import frc.robot.hardware.motors.TalonFXIO;
import frc.robot.utilities.FeedbackController.PIDFeedback;

public class SwerveConstants {
    public static final SwerveModule FRONT_LEFT_MODULE = SwerveModule.fromMeasuredValues(
        new Translation2d(0.368, 0.266), 
        0.0467733641327, 
        new TalonFXIO(
            "Swerve/Front Left Drive", 
            SwerveWiring.FRONT_LEFT_DRIVE_ID, 
            new TalonFXConfiguration(), 
            DCMotor.getKrakenX60(1)
        ), 
        new PIDFeedback(1.96850393701), 
        5.143, 
        1, 
        0.19635,
        0.10308336,
        0.009937496, 
        new SparkMaxIO(
            "Swerve/Front Left Angle", 
            SwerveWiring.FRONT_LEFT_ANGLE_ID, 
            new SparkMaxConfig(), 
            DCMotor.getNeo550(1)
        ), 
        new PIDFeedback(5.72957795131), 
        25, 
        1, 
        0.25348, 
        0.491242554389, 
        0.0619252784977, 
        new AnalogEncoderIO("Swerve/Front Left Encoder", 
            SwerveWiring.FRONT_LEFT_ENCODER_ID, 
            0.642, 
            encoder -> {}
        )
    );
    
    public static final SwerveModule FRONT_RIGHT_MODULE = SwerveModule.fromMeasuredValues(
        new Translation2d(0.368, -0.266), 
        0.0467733641327, 
        new TalonFXIO(
            "Swerve/Front Right Drive", 
            SwerveWiring.FRONT_RIGHT_DRIVE_ID, 
            new TalonFXConfiguration(), 
            DCMotor.getKrakenX60(1)
        ), 
        new PIDFeedback(1.96850393701), 
        5.143, 
        1, 
        0.20427,
        0.10233152,
        0.012937236, 
        new SparkMaxIO(
            "Swerve/Front Left Angle", 
            SwerveWiring.FRONT_LEFT_ANGLE_ID, 
            new SparkMaxConfig(), 
            DCMotor.getNeo550(1)
        ), 
        new PIDFeedback(5.72957795131), 
        25, 
        1, 
        0.27701, 
        0.515003114153, 
        0.0627675264566, 
        new AnalogEncoderIO("Swerve/Front Right Encoder", 
            SwerveWiring.FRONT_RIGHT_ENCODER_ID, 
            0.668, 
            encoder -> {}
        )
    );

    public static final SwerveModule BACK_LEFT_MODULE = SwerveModule.fromMeasuredValues(
        new Translation2d(-0.368, 0.266), 
        0.0467733641327, 
        new TalonFXIO(
            "Swerve/Back Left Drive", 
            SwerveWiring.BACK_LEFT_DRIVE_ID, 
            new TalonFXConfiguration(), 
            DCMotor.getKrakenX60(1)
        ), 
        new PIDFeedback(1.96850393701), 
        5.143, 
        1, 
        0.2049,
        0.10245852,
        0.01343152, 
        new SparkMaxIO(
            "Swerve/Back Left Angle", 
            SwerveWiring.BACK_LEFT_ANGLE_ID, 
            new SparkMaxConfig(), 
            DCMotor.getNeo550(1)
        ), 
        new PIDFeedback(5.72957795131), 
        25, 
        1, 
        0.25886, 
        0.520658207591, 
        0.0725479160195, 
        new AnalogEncoderIO("Swerve/Back Left Encoder", 
            SwerveWiring.BACK_LEFT_ENCODER_ID, 
            0.022, 
            encoder -> {}
        )
    );
    
    public static final SwerveModule BACK_RIGHT_MODULE = SwerveModule.fromMeasuredValues(
        new Translation2d(-0.368, -0.266), 
        0.0467733641327, 
        new TalonFXIO(
            "Swerve/Back Right Drive", 
            SwerveWiring.BACK_RIGHT_DRIVE_ID, 
            new TalonFXConfiguration(), 
            DCMotor.getKrakenX60(1)
        ), 
        new PIDFeedback(1.96850393701), 
        5.143, 
        1, 
        0.20206,
        0.10634472,
        0.009241536, 
        new SparkMaxIO(
            "Swerve/Back Right Angle", 
            SwerveWiring.BACK_RIGHT_ANGLE_ID, 
            new SparkMaxConfig(), 
            DCMotor.getNeo550(1)
        ), 
        new PIDFeedback(5.72957795131), 
        25, 
        1, 
        0.25348, 
        0.528765560392, 
        0.0818699393462, 
        new AnalogEncoderIO("Swerve/Back Right Encoder", 
            SwerveWiring.BACK_RIGHT_ENCODER_ID, 
            0.879, 
            encoder -> {}
        )
    );
}
