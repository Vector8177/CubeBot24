package frc.robot.subsystems.swerve;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Module[] mSwerveMods;

    private GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private ChassisSpeeds chassisSpeeds;

    private Field2d field;

    public Swerve(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {

        this.gyroIO = gyroIO;
        setYaw(Rotation2d.fromDegrees(0));

        mSwerveMods =
                new Module[] {
                    new Module(flModuleIO, 0),
                    new Module(frModuleIO, 1),
                    new Module(blModuleIO, 2),
                    new Module(brModuleIO, 3)
                };

        swervePoseEstimator =
                new SwerveDrivePoseEstimator(
                        SwerveConstants.swerveKinematics,
                        getYaw(),
                        getPositions(),
                        new Pose2d(),
                        SwerveConstants.STATE_STANDARD_DEVIATIONS,
                        VecBuilder.fill(0, 0, 0));

        resetToAbsolute();

        field = new Field2d();
        chassisSpeeds = new ChassisSpeeds();
        SmartDashboard.putData(field);
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.fastSpeedLimit);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.index], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.fastSpeedLimit);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.index], false);
        }
    }

    public void setModuleRotation(Rotation2d rotation) {
        for (Module mod : mSwerveMods) {
            mod.setDesiredState(new SwerveModuleState(0, rotation), false);
        }
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public void resetOdometry(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetToAbsolute() {
        for (Module mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module mod : mSwerveMods) {
            states[mod.index] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module mod : mSwerveMods) {
            positions[mod.index] = mod.getPosition();
        }
        return positions;
    }

    public void setYaw(Rotation2d angle) {
        gyroIO.setYaw(angle); // Angle in Degrees
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyroInputs.pitchPosition);
    }

    public PathPoint getPoint() {
        return PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyroInputs.yawPosition)
                : Rotation2d.fromDegrees(gyroInputs.yawPosition);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyroInputs.rollPosition);
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (Module mod : mSwerveMods) {
            mod.periodic();
        }

        swervePoseEstimator.update(getYaw(), getPositions());

        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        Logger.getInstance().recordOutput("Odometry/RobotPose", getPose());
        Logger.getInstance().recordOutput("SwerveModuleStates", getStates());
    }
}
