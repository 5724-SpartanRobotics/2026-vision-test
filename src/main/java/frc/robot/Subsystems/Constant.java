package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Constant {
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;

    public static int signum(double x) { return x >= 0 ? 1 : -1; }
    public static int signum(int x) { return x >= 0 ? 1 : -1; }

    public static boolean isClose(double number, double to, double tolerance) { return Math.abs(number - to) <= tolerance;  }
    public static boolean isClose(int number, int to, int tolerance) { return Math.abs(number - to) <= tolerance;  }

    public static class HelixPIDController {
        public double kP;
        public double kD;
        private double kI;
        private double d;
        private double i;
        private double lastError;
        public double reference;
        public double inputRange;
        public boolean continuous;

        public HelixPIDController(double P, double I, double D) {
            this.kP = P; this.kI = I; this.kD = D;
            i = 0; lastError = 0;
            inputRange = Double.POSITIVE_INFINITY;
            continuous = false;
        }

        public double setP(double P) {
            kP = P;
            return kP;
        }

        public double calculate(double state, double dt) {
            double error = (reference - state) % inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) error -= inputRange;
                else error += inputRange;
            }
            if (dt > 0) {
                d = (error - lastError) / dt;
                i += error *dt;
            }
            lastError = error;
            return kP * error + kI * i + kD * d;
        }

        public void setReference(double ref) {
            reference = ref;
        }

        public void setContinuous(boolean cont) {
            continuous = cont;
        }

        public void setRange(double r) {
            inputRange = r;
        }
    }

    public static abstract class LockonSubsystem extends SubsystemBase {
        public static class TagReading {
            public double distance;
            public double angle;
    
            public TagReading(double dist, double angle) {
                this.distance = dist;
                this.angle = angle;
            }
        }

        public LockonSubsystem() {
            super();
        }

        /**
         * @param driveTrain
         * @param targetYaw
         * @param speedMod
         * @return Returns lastTheta.
         */
        public double _drive(DriveTrainSubsystem driveTrain, double speedMod) {
            double targetDistance = 0; // PARAM
            String tagName = "36h11"; // PARAM

            double distance_ft = getDistance();
            if (distance_ft != Double.NEGATIVE_INFINITY) {
                double theta = -getTheta() + driveTrain.getGyroHeading().getDegrees() - /* AutoConstants.cameraAngleOffset */ 0;
                double delta_d_m = Units.feetToMeters(distance_ft - targetDistance + /* AutoConstants.cameraDepthOffset */ 0);
                double delta_x = -delta_d_m * Math.sin(Units.degreesToRadians(theta));
                double delta_y = delta_d_m * Math.cos(Units.degreesToRadians(theta));
                Translation2d dTranslation = new Translation2d(-delta_y, delta_x);
                Pose2d pose = new Pose2d(dTranslation, driveTrain.getGyroHeading().times(-1F));
                driveTrain.ZeroDriveSensors(pose);
                return theta;
            } else {
                Translation2d dTranslation = new Translation2d(0, 0);
                Pose2d pose = new Pose2d(dTranslation, driveTrain.getGyroHeading().times(-1F));
                driveTrain.ZeroDriveSensors(pose);
                return driveTrain.getGyroHeading().getDegrees();
            }
        };
        public double drive(DriveTrainSubsystem driveTrain, Double speedMod) {
            double speed = speedMod == null ? 1 : speedMod;
            double photonAngle = Units.degreesToRadians(getTheta());
            double gyroAngle = driveTrain.getGyroHeading().getRadians();
            double fX = Math.sin(gyroAngle) * speed;
            double fY = Math.cos(gyroAngle) * speed;
            double sV = -speed * photonAngle * 0.005;
            double sX = Math.sin(gyroAngle + HalfPI) * sV;
            double sY = Math.cos(gyroAngle + HalfPI) * sV;
            Translation2d t2d = new Translation2d(fY + sY , fX + sX);
            driveTrain.drive(t2d, 0);
            SmartDashboard.putNumber("photonAngle", photonAngle);
            SmartDashboard.putNumber("gyroAngle", gyroAngle);
            return photonAngle;
        };
        public Double getDistance() {
            return 0D;
        };
        public double getTheta() {
            return 0F;
        };
        public TagReading getReading() {
            return new TagReading(getDistance(), getTheta());
        }
    }

    public static final class CanIdConstants{
        public static final int LFTurnMotor = 2;//falcon500
        public static final int LFDriveMotor = 3;//falcon500
        public static final int LFCanID = 10;
        public static final int LBTurnMotor = 4;//falcon500
        public static final int LBDriveMotor = 5;//falcon500
        public static final int LBCanID = 11;
        public static final int RBTurnMotor = 6;//falcon500
        public static final int RBDriveMotor = 7;//falcon500
        public static final int RBCanID = 13;
        public static final int RFTurnMotor = 8;//falcon500
        public static final int RFDriveMotor = 9;//falcon500
        public static final int RFCanID = 12;
        public static final int PigeonID = 14;
        public static final int PDHID = 0;
        public static final int pneumaticontroller = 15;
     }
     public final class DriveConstants {
        // Maximum robot speed (in meters per second)
        public static final double maxRobotSpeedmps = 4.5;
        public static final double driveGearRatio = 6.75;
        public static double turnGearRatio = 150.0 / 7.0;
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
    
        // Maximum angular velocity (in radians per second)
        public static final double maxAngularVelocityRadps = Math.PI * 2;
        public static final double maxLinearSpeed = 15.5F / 3.281F;
    
        // Wheelbase and track width (in meters)
        public static final double wheelBase = 0.6; // Distance between front and back wheels
        public static final double trackWidth = 0.6; // Distance between left and right wheels
    
        // Swerve module offsets (in radians) - adjust these based on calibration
        public static final double LFOff = 0.0;
        public static final double RFOff = 0.0;
        public static final double LBOff = 0.0;
        public static final double RBOff = 0.0; 
    
        // Module positions relative to the robot center (in meters)
        public static final Translation2d LFLocation = new Translation2d(wheelBase / 2.0, trackWidth / 2.0); // Front left
        public static final Translation2d RFLocation = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0); // Front right
        public static final Translation2d LBLocation = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0); // Back left
        public static final Translation2d RBLocation = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0); // Back right
    
        // PID Controller tuning values for trajectory tracking
        public static final double xPID = 10.0;
        public static final double yPID = 10.0;
        public static final double headingPID = 7.5;
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
        public static double joystickZDeadband = 0.3;//a deadband that you must overcome for the joystick input, otherwise we send 0

        public static final class ButtonMap {
            public static final int GyroZero = 7;
            public static final int TagLockon = 1;
            public static final int TagLockonAlt = 11;
            public static final int ObjectLockon = 2;
            public static final int ObjectLockonAlt = 12;
            public static final int OuttakePiece = -1;
            public static final int Rotate180 = 8;
        }
    }
    public static final class DebugSetting{
        //set this to get more values to smart dashboard. Leave it off for competition.
        public static final DebugLevel TraceLevel = DebugLevel.Swerve;
    }
    public static enum DebugLevel{
        Off,
        Swerve,
        Amp,
        Climber,
        IntakeShooter,
        Helix,
        Vision,
        All
    }
    public static final class AutoConstants {
        public static final double kPTranslationController = 2;
        public static final double kPThetaController = 3;
        public static final double autoShootCloseness = 0.003;
        public static final double degreesError = 5;
        public static final double cameraAngleOffset = Math.PI / 6.5;
        public static final double cameraDepthOffset = 1;
        public static final double kPAutoShoot = 6;
        public static final double kPTurnAutoShoot = 12;

        public static final String kLimelightName = "limelight";
    }

    public static final class CameraConstants {
        public static final class MicrosoftLifeCamHD3000 {
            public static final int Width_px = 640 / 2;
            public static final int Height_px = 480 / 2;
            public static final int vFOV = 60;
            public static final int hFOV = 120;
        }

        public static final class Limelight2Plus {
            public static final double HeightFromFloor = 0.3;
            public static final double MountAngleOffset = -0.261799;
        }
    }
}
