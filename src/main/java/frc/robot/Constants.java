package frc.robot;

// import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj.RobotBase;

/*
 * General Robot Constants
 * - For specific constants look in subsystem folder
 */
public final class Constants {

    public static Mode getMode() {
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public enum Mode {
        /** Replaying from a log file. */
        REAL,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum Position {
        HIGH(0, 0),
        CONEHIGH(.15, 34.5),
        CONEHIGHUP(1.55, 34.5),
        CUBEHIGH(1.55, 34),
        MID(0, 0),
        CONEMID(5.81731, 35),
        CUBEMID(1.427, 16.5),
        LOW(0, 0),
        CONELOW(-0.15, 1.5),
        CUBELOW(1.425, .25),
        STANDBY(1.1765, .25),
        CUBEINTAKE(-0.05, 0.25),
        STANDINGCONEINTAKE(5.106, 14.083),
        TIPPEDCONEINTAKE(5.572, 1.333),
        HUMANPLAYERINTAKE(.8763, 1.5),
        DOUBSUBSTATIONINTAKE(6.0076, 33.178);

        private double wristPos;
        private double elevatorPos;

        private Position(double wrist, double elev) {
            this.wristPos = wrist;
            this.elevatorPos = elev;
        }

        public double getWrist() {
            return wristPos;
        }

        public double getElev() {
            return elevatorPos;
        }
    }

    public enum GamePiece {
        CUBE(1),
        CONE(-1);

        private double direction;

        private GamePiece(double value) {
            direction = value;
        }

        public double getDirection() {
            return direction;
        }
    }

    public enum SEGMENT { // Numbers in order of segment from left to right (driver station POV)
        CONE_1(0),
        CONE_2(31.8),
        CONE_3(35.2),
        CONE_4(-1),
        CONE_5(-1),
        CONE_6(-1),
        CUBE_1(0),
        CUBE_2(20.6),
        CUBE_3(35.2),
        HUMANPLAYER(-1);

        // intake Ground Cube: 0
        // intake Cone Upright: 12
        // intake Cone Tipped: 0

        // intake Cone Single HP: 8.9
        // intake Cube Single HP: 11.8

        private double level;

        private SEGMENT(double level) {
            this.level = level;
        }

        public static SEGMENT getSegment(int level, boolean cone) {
            if (cone) {
                switch (level) {
                    case 1:
                        return CONE_1;
                    case 2:
                        return CONE_2;
                    case 3:
                        return CONE_3;
                }
            } else {
                switch (level) {
                    case 1:
                        return CUBE_1;
                    case 2:
                        return CUBE_2;
                    case 3:
                        return CUBE_3;
                }
            }
            return null;
        }

        public double getValue() {
            return level;
        }
    }

    public static final class Autonomous {
        // public static final PathConstraints constraints = new PathConstraints(1, 1);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.8;
        public static final double kPYController = 1.8;
        public static final double kPThetaController = 1.9;

        /* Constants for line up */
        public static final double kPGridLineUp = 0.8;
        public static final double kIGridLineUp = 0.0;
        public static final double gridLineUpTolerance = 0.05;

        public static final double kPThetaGridLineUp = 0.025;
        public static final double thetaGridLineUpTolerance = 2.0;

        public static final double gridLineUpAngle = 180.0;
        public static final double gridLineUpStrafePosition = 7.35;
    }
}
