package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GamePiece;
// import frc.robot.Constants.Position;
// import frc.robot.autos.AutoBalancing;
import frc.robot.commands.*;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSparkMax;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
// import java.util.HashMap;

public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private static final int translationAxis = XboxController.Axis.kLeftY.value;
    private static final int strafeAxis = XboxController.Axis.kLeftX.value;
    private static final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Swerve s_Swerve;

    public static GamePiece gamePiece = GamePiece.CONE;

    public RobotContainer() {
        switch (Constants.getMode()) {
            case REAL:
                s_Swerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOSparkMax(SwerveConstants.Mod0.constants),
                        new ModuleIOSparkMax(SwerveConstants.Mod1.constants),
                        new ModuleIOSparkMax(SwerveConstants.Mod2.constants),
                        new ModuleIOSparkMax(SwerveConstants.Mod3.constants));
                break;

            default:
                s_Swerve = new Swerve(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                break;
        }

        CameraServer.startAutomaticCapture();

        // Sets each subsystem's default commands
        setDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();

        // Configure Smart Dashboard options
        configureSmartDashboard();

        // Configure autonomous events
        configureAutonomousEvents();

        // autoBuilder =
        // new SwerveAutoBuilder(
        // s_Swerve::getPose,
        // s_Swerve::resetOdometry,
        // SwerveConstants.swerveKinematics, // SwerveDriveKinematics
        // new PIDConstants(Constants.Autonomous.kPXController, 0, 0),
        // new PIDConstants(Constants.Autonomous.kPThetaController, 0, 0),
        // s_Swerve::setModuleStates,
        // eventMap,
        // true,
        // s_Swerve);
    }

    private void setDefaultCommands() {
        // Set To Field Orientation When Robot Turns On
        s_Swerve.setYaw(Rotation2d.fromDegrees(180));

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver.povDown().getAsBoolean(),
                        () -> driver.leftBumper().getAsBoolean(),
                        () -> driver.rightBumper().getAsBoolean(),
                        () -> driver.a().getAsBoolean()));
    }

    private void configureAutonomousEvents() {

        // eventMap.put("wait1Seconds", new WaitCommand(1));

        // eventMap.put("AutoBalance", new AutoBalancing(s_Swerve, true));

        // eventMap.put("AutoBalanceStraight", new AutoBalancing(s_Swerve, false));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     *
     * <p>
     * This method binds the buttons to commands.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.setYaw(Rotation2d.fromDegrees(0))));

        /*
         * driver.b().whileTrue(
         * autoBuilder.followPath(
         * segmentLineUp.getTrajectory(SEGMENT.HUMANPLAYER,
         * () -> s_Swerve.getPose())));
         */

        /*
         * operator.x().onTrue(new SelectCommand(
         * Map.ofEntries(
         * Map.entry(GamePiece.CUBE,
         * new TimedIntake(s_Intake, .5, GamePiece.CUBE,
         * EjectSpeed.CUBEFAST, Direction.OUTTAKE)),
         * Map.entry(GamePiece.CONE,
         * new TimedIntake(s_Intake, .5, GamePiece.CONE,
         * EjectSpeed.CONEFAST, Direction.OUTTAKE))),
         * () -> gamePiece));
         */
    }

    private void configureSmartDashboard() {
        // Autonomous Mode Chooser
        // autoChooser.addOption("Clean: Score 2 Plus Balance", twoPlusBalance);
        // autoChooser.addOption("Clean: 3 Game Piece Auto", threePieceAuto);
        // autoChooser.addOption("Clean: Score 2 Plus Field", twoPlusPickup);

        // autoChooser.addOption("Center: Cube Balance", autoBalance);
        // autoChooser.addOption("Center: Cone Mobility Balance", coneMobilBalance);
        // autoChooser.addOption("Center: Cube Mobility Balance", cubeMobilBalance);

        // autoChooser.addOption("Bump: 2 Piece", bump2Piece);
        // autoChooser.addOption("Bump: 3 Piece", bump3Piece);
        // autoChooser.addOption("Bump: 3 Piece Alt", bump3PieceAlt);

        // autoChooser.addDefaultOption("Cone PickupCube Balance", coneCubeBalance);
        // autoChooser.addOption("Back and Forth", backnForth);
        // autoChooser.addOption("S curve", sCurve);
        // autoChooser.addOption("L3 Cone+Cube", coneCubeDeposit);
    }

    /** Ran once the robot is put in disabled */
    public void disabledPeriodic() {
    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece piece) {
        gamePiece = piece;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // s_Swerve.setYaw(Rotation2d.fromDegrees(180));
    // Logger.getInstance().recordOutput("Trajectory", autoChooser.get());

    // // Executes the autonomous command chosen in smart dashboard
    // return new ParallelCommandGroup(
    // new InstantCommand(
    // () ->
    // s_Swerve.getField().getObject("Field").setTrajectory(autoChooser.get())),
    // autoBuilder.fullAuto(autoChooser.get()));
    // }/
}
