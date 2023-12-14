package frc.lib.config;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class CTREConfigs {
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        swerveCanCoderConfig.sensorDirection = SwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
