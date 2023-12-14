package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class SwerveConstants {
    public static final double stickDeadband = 0.1;

    public static final Vector<N3> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.02, 0.02, 0.005);

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.12:1
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

    public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30;
    public static final int driveContinuousCurrentLimit = 40;

    public static final double pitchSetPoint = 0.0;

    public static final double drivePitchKP = 0.015;
    public static final double drivePitchKI = 0.0001;
    public static final double drivePitchKD = 1e-15; // 1 in the 15th decimal place
    public static final double drivePitchKFF = 1e-15;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.16059;
    public static final double driveKV = 2.4135;
    public static final double driveKA = 0.41807;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double speedLimit = 4.5; // Base Meters Per Second Speed
    public static final double slowSpeedLimit = 1.0; // Slow Meters Per Second Speed
    public static final double fastSpeedLimit = 4.5; // Fast Meters Per Second Speed

    public static final double accelerationLimit = 3.5;
    public static final double fastAccelerationLimit = 10.0;

    public static final double angularVelocityLimit = 5;
    public static final double slowAngularVelocityLimit = 1.25;
    public static final double fastAngularVelocityLimit = 6;

    /* Swerve Limiting Values */
    public static final double autoCenterLimit = .3;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 01;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 12;
        public static final int angleMotorID = 02;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 13;
        public static final int angleMotorID = 03;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 14;
        public static final int angleMotorID = 04;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }
}
