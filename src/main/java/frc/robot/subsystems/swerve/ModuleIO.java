package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double absoluteEncoder = 0.0;

        public double drivePosition = 0.0;
        public double driveVelocityPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        public double turnAbsolutePosition = 0.0;
        public double turnPosition = 0.0;
        public double turnVelocityPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void resetToAbsolute() {}

    public default void setMotorOutput(double percentOutput) {}

    public default void setVelocity(SwerveModuleState desiredState, double ffVoltage) {}

    public default void setAngle(Rotation2d angle) {}
}
