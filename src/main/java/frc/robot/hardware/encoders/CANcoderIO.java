package frc.robot.hardware.encoders;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class CANcoderIO extends AbsoluteEncoderIO {
    private CANcoder encoder;
    public CANcoderIO(String name, int deviceID, Consumer<CANcoder> config) {
        this(name, deviceID, new CANBus(), config);
    }

    public CANcoderIO(String name, int deviceID, CANBus canbus, Consumer<CANcoder> config) {
        super(name);
        encoder = new CANcoder(deviceID, canbus);
        config.accept(encoder);
    }

    public CANcoderIO(String name, int deviceID, CANcoderConfiguration config) {
        this(name, deviceID, new CANBus(), config);
    }

    public CANcoderIO(String name, int deviceID, CANBus canbus, CANcoderConfiguration config) {
        this(name, deviceID, canbus, CANcoder -> {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                CANcoder.getConfigurator().apply(config);
            }
        });
    }

    @Override
    public void updateInputs() {
        inputs.position = Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble());
        Logger.processInputs(name, inputs);
    }

    @Override
    public void updateSim(double velocity, double dt) {
        encoder.getSimState().setRawPosition(Units.radiansToRotations(inputs.position +  velocity * dt));
    }

}
