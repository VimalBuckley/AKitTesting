package frc.robot.hardware.encoders;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;

public class AnalogEncoderIO extends AbsoluteEncoderIO {
    private AnalogEncoder encoder;
    private double offset;
    private double simPosition;

    public AnalogEncoderIO(String name, int channel, double offset, Consumer<AnalogEncoder> config) {
        super(name);
        encoder = new AnalogEncoder(channel);
        config.accept(encoder);
        this.offset = offset;
    }

    @Override
    public void updateInputs() {
        if (RobotBase.isReal()) {
            inputs.position = encoder.get() - offset;
        } else {
            inputs.position = simPosition;
        }
        Logger.processInputs(name, inputs);
    }

    @Override
    public void updateSim(double velocity, double dt) {
        simPosition += velocity * dt;
    }
    
}
