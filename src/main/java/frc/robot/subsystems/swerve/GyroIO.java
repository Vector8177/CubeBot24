package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double rollPosition = 0.0;
        public double pitchPosition = 0.0;
        public double yawPosition = 0.0;
        public double rollVelocityPerSec = 0.0;
        public double pitchVelocityPerSec = 0.0;
        public double yawVelocityPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setYaw(Rotation2d angle) {}
}
