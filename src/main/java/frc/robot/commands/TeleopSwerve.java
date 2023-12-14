package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.VectorTools.util.SlewRateLimiter;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftBumper;
    private BooleanSupplier rightBumper;
    private BooleanSupplier gridLineUp;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    // private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private PIDController strafeController;
    private PIDController rotationController;

    private enum Speed {
        FAST,
        NORMAL,
        SLOW
    }

    /**
     * The constructor initializes the class variables.
     *
     * @param s_Swerve
     * @param translationSup
     * @param strafeSup
     * @param rotationSup
     * @param autoCenter
     * @param robotCentricSup
     */
    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier leftBumper,
            BooleanSupplier rightBumper,
            BooleanSupplier gridLineUp) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.gridLineUp = gridLineUp;
    }

    @Override
    public void initialize() {
        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);

        strafeController =
                new PIDController(Constants.Autonomous.kPGridLineUp, Constants.Autonomous.kIGridLineUp, 0);
        strafeController.setTolerance(Constants.Autonomous.gridLineUpTolerance);

        rotationController = new PIDController(Constants.Autonomous.kPThetaGridLineUp, 0, 0);
        rotationController.setTolerance(Constants.Autonomous.thetaGridLineUpTolerance);
        rotationController.enableContinuousInput(0, 360);
    }

    /** TODO */
    @Override
    public void execute() {
        /* Set Speeds based on button input */
        Speed speed =
                leftBumper.getAsBoolean()
                        ? Speed.SLOW
                        : rightBumper.getAsBoolean() ? Speed.FAST : Speed.NORMAL;

        double speedLimit = SwerveConstants.speedLimit;
        double angularSpeedLimit = SwerveConstants.angularVelocityLimit;

        switch (speed) {
            case FAST:
                translationLimiter.setRateLimit(SwerveConstants.fastAccelerationLimit);
                strafeLimiter.setRateLimit(SwerveConstants.fastAccelerationLimit);

                speedLimit = SwerveConstants.fastSpeedLimit;
                angularSpeedLimit = SwerveConstants.fastAngularVelocityLimit;
                break;
            case SLOW:
                translationLimiter.setRateLimit(SwerveConstants.accelerationLimit);
                strafeLimiter.setRateLimit(SwerveConstants.accelerationLimit);

                speedLimit = SwerveConstants.slowSpeedLimit;
                angularSpeedLimit = SwerveConstants.slowAngularVelocityLimit;
                break;
            default:
                translationLimiter.setRateLimit(SwerveConstants.accelerationLimit);
                strafeLimiter.setRateLimit(SwerveConstants.accelerationLimit);
                break;
        }

        /* Get Values, Deadband */
        double translationVal =
                MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal;
        double strafeVal;

        if (gridLineUp.getAsBoolean()) {
            strafeVal =
                    MathUtil.clamp(
                            strafeController.calculate(
                                    s_Swerve.getPose().getY(), Constants.Autonomous.gridLineUpStrafePosition),
                            -1,
                            1);

            rotationVal =
                    MathUtil.clamp(
                            rotationController.calculate(
                                    s_Swerve.getYaw().getDegrees(), Constants.Autonomous.gridLineUpAngle),
                            -1,
                            1);

            if (strafeController.atSetpoint()) strafeVal = 0;

        } else {

            rotationVal =
                    MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband);
        }

        s_Swerve.drive(
                new Translation2d(
                                translationLimiter.calculate(translationVal), strafeLimiter.calculate(strafeVal))
                        .times(speedLimit),
                rotationVal * (angularSpeedLimit),
                !robotCentricSup.getAsBoolean(),
                false);
    }
}
