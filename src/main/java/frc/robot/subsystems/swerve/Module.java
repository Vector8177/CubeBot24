package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
    private Rotation2d lastAngle;

    public ModuleIO io;

    public int index;

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        lastAngle = Rotation2d.fromDegrees(inputs.turnPosition);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    /**
     * @param desiredState
     * @param isOpenLoop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState =
                SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(inputs.turnPosition));
        // Custom optimize command, since default WPILib optimize assumes
        // continuous controller which REV and CTRE are not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * @param desiredState
     * @param isOpenLoop
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.fastSpeedLimit;
            io.setMotorOutput(percentOutput);
        } else {
            io.setVelocity(desiredState, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * @param desiredState
     */
    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond)
                                <= (SwerveConstants.fastAngularVelocityLimit * 0.01))
                        ? lastAngle
                        : desiredState.angle;

        io.setAngle(angle);
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        io.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                inputs.driveVelocityPerSec, Rotation2d.fromDegrees(inputs.turnPosition));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                inputs.drivePosition, Rotation2d.fromDegrees(inputs.turnPosition));
    }
}
