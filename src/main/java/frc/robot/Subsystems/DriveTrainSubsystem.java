package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.CanIdConstants;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.commands.TeleopSwerve;

public class DriveTrainSubsystem extends SubsystemBase implements DriveTrainInterface {
        // Swerve modules
        private boolean parkFlag;
        private boolean doneFlag;

        private SwerveModule LF;
        private SwerveModule RF;
        private SwerveModule LB;
        private SwerveModule RB;
        private Rotation2d lastUpdatedGyroHeading;
    
        // Used by helix, should eventually be used to zero gyro with a button
        private double offset = 0;

        // Gyro for now
        // private ADIS16448_IMU gyro;
        // private ADIS16448_IMUSim gyroSim;
  //      private ADXRS450_Gyro gyroFake;
        private ADXRS450_GyroSim gyroSim;
        private Pigeon2 gyro;

        // TODO make pigeon sim actually like, work

        private SwerveDriveKinematics swerveDriveKinematics;
        private SwerveDriveOdometry swerveDriveOdometry;
        private Pose2d robotPose;
        private SwerveModule[] modules;
        private boolean spinFlag;
    
        public DriveTrainSubsystem() {
//            gyro = new ADIS16448_IMU();
            gyro = new Pigeon2(CanIdConstants.PigeonID);
            UpdateGyro();
            LF = new SwerveModule(CanIdConstants.LFTurnMotor, CanIdConstants.LFDriveMotor, CanIdConstants.LFCanID, DriveConstants.LFOff, "LF", this);
            RF = new SwerveModule(CanIdConstants.RFTurnMotor, CanIdConstants.RFDriveMotor, CanIdConstants.RFCanID, DriveConstants.RFOff, "RF", this);
            LB = new SwerveModule(CanIdConstants.LBTurnMotor, CanIdConstants.LBDriveMotor, CanIdConstants.LBCanID, DriveConstants.LBOff, "LB", this);
            RB = new SwerveModule(CanIdConstants.RBTurnMotor, CanIdConstants.RBDriveMotor, CanIdConstants.RBCanID, DriveConstants.RBOff, "RB", this);
            swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.LFLocation, DriveConstants.RFLocation, DriveConstants.LBLocation, DriveConstants.RBLocation);
            robotPose = new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d());//starting pose doesn't really matter. We will call reset based on robot initial field position.
            SwerveModulePosition[] swerveInititialPositions = {LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()};
            swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroHeading(), swerveInititialPositions, robotPose);
            modules = new SwerveModule[] {LF, RF, LB, RB};
            parkFlag = false;
            doneFlag = false;
        }

        public Rotation2d getGyroHeading(){
            return lastUpdatedGyroHeading;
        }

        public void setGyroZero() {
             gyro.setYaw(0);
        }

        public void ZeroDriveSensors(Pose2d xy) {
            // LF.ZeroDriveSensor();
            // RF.ZeroDriveSensor();
            // RB.ZeroDriveSensor();
            // LB.ZeroDriveSensor();
            //zero the robot pose
            //System.out.println("old pose " + swerveDriveOdometry.getPoseMeters().getX() + ", " + swerveDriveOdometry.getPoseMeters().getY());
            swerveDriveOdometry.resetPosition(lastUpdatedGyroHeading, new SwerveModulePosition[] {LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()}, xy.times(-1));
            //System.out.println("new pose " + swerveDriveOdometry.getPoseMeters().getX() + ", " + swerveDriveOdometry.getPoseMeters().getY());
        }

        @Override
        public void periodic(){
            super.periodic();
            Rotation3d gyroPosition = gyro.getRotation3d();
            if (DebugSetting.TraceLevel == DebugLevel.All || DebugSetting.TraceLevel == DebugLevel.Swerve){
                //SmartDashboard.putNumber("Xpos", -getPose().getX());
                //SmartDashboard.putNumber("Ypos", -getPose().getY());
                //SmartDashboard.putNumber("Zpos", -getPose().getRotation().getRadians());
                //SmartDashboard.putNumber("roll", gyroPosition.getX()); //x=roll, y=pitch, z=yaw
                //SmartDashboard.putNumber("pitch", gyroPosition.getY());
            }
            //the gyro getGyroAngleY returns positive values as the robot turns clockwise. We want negative for clockwise
            LF.periodic();
            LB.periodic();
            RF.periodic();
            RB.periodic();
            if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
                SmartDashboard.putNumber("Gyro Heading Deg", getGyroHeading().getDegrees());
            }
            UpdateGyro();
            robotPose = swerveDriveOdometry.update(getGyroHeading(), new SwerveModulePosition[] {LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()});
            if (DebugSetting.TraceLevel == DebugLevel.Swerve){
                SmartDashboard.putNumber("RobotPoseX", robotPose.getX());
                SmartDashboard.putNumber("RobotPoseY", robotPose.getY());
            }

        }

      private void UpdateGyro() {
          lastUpdatedGyroHeading =
        Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
}

        public void flipGyro() {
            gyro.setYaw(Math.PI);
            System.out.println("gyro: " + gyro.getYaw());
        }

        // public void drive(Translation2d translation, double rotation){
        //     // translation.times(1);
        //     SwerveModuleState[] swerveModStates = swerveDriveKinematics.toSwerveModuleStates(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading()));
        //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);
        //     LF.setDesiredState(swerveModStates[0]);
        //     RF.setDesiredState(swerveModStates[1]);
        //     LB.setDesiredState(swerveModStates[2]);
        //     RB.setDesiredState(swerveModStates[3]);
        // }

        public void simulationInit()
        {
//            gyroSim = new ADIS16448_IMUSim(gyro);
//            gyroSim = new ADXRS450_GyroSim(gyroFake);
         //   LF.simulateInit();
         //   RF.simulateInit();
         //   LB.simulateInit();
         //   RB.simulateInit();
//          gyroSim.setGyroAngleY(0);
//            gyroSim.setAngle(0);
        }
        @Override
        public void simulationPeriodic(){
            if (gyroSim == null)
                simulationInit();
//            gyroSim.setGyroAngleY(Units.radiansToDegrees(0.0));
//            gyroSim.setAngle(0.0);
         //   LF.simulatePeriodic();
         //   RF.simulatePeriodic();
         //   LB.simulatePeriodic();
         //   RB.simulatePeriodic();
            }

            // used by helix
        public void brake() {
            for (SwerveModule module : modules) {
                module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
            }
        }
        public Pose2d getPose() {
            return robotPose;
        }

        public void setFlag() {
            parkFlag = false;
            doneFlag = false;
            spinFlag = false;
        }

        public void heyAreWeUpYet() {
            if(doneFlag) {

            } else if(Math.abs(gyro.getRoll().getValueAsDouble()) < 10 && parkFlag){
                //System.out.println("up");
                this.drive(new Translation2d(0, 0.1), 0);
                doneFlag = true;
            }else {
            // }else if(Math.abs(gyro.getRoll()) > 10.5) {
                this.drive(new Translation2d(-0.8, 0), 0);
                //System.out.println("flag");
                parkFlag = true;
            }
            // } else {
            //     this.drive(new Translation2d(-2, 0), 0);
            //     //System.out.println("back");
            // }
        }

        public void yeet(int inv) {
            if(Math.abs(Math.abs(this.getGyroHeading().getDegrees()) % 180 - 90) < 10) {
             //   armForAuto.Run(0.5 * 40 * 6000);
        
            }

        }

        public void spiiiiiiin(double target, int inv) {
            if(Math.abs((int)(this.getGyroHeading().getDegrees()) % 360 - target) < 10) {
                spinFlag = true;
            }
            if(!spinFlag) {
                this.drive(new Translation2d(), 0.5 * DriveConstants.maxRobotSpeedmps * inv);
            } else {
                this.drive(new Translation2d(), 0);
            }
        }

            protected void _drive(Translation2d translation, double rotation) {
        ChassisSpeeds speeds = true ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroHeading()
            ) :
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] swerveModStates = swerveDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);

        LF.setDesiredState(swerveModStates[0]);
        RF.setDesiredState(swerveModStates[1]);
        LB.setDesiredState(swerveModStates[2]);
        RB.setDesiredState(swerveModStates[3]);
    }

    public DriveTrainSubsystem drive(SwerveModuleState[] swerveModStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);

        LF.setDesiredState(swerveModStates[0]);
        RF.setDesiredState(swerveModStates[1]);
        LB.setDesiredState(swerveModStates[2]);
        RB.setDesiredState(swerveModStates[3]);
        return this;
    }

    public DriveTrainSubsystem drive(ChassisSpeeds speeds) {
        return drive(swerveDriveKinematics.toSwerveModuleStates(speeds));
    }

    public DriveTrainSubsystem drive(Translation2d translation, double rotation) {
        _drive(translation, rotation);
        return this;
    }

    public DriveTrainSubsystem drive(double rotation) {
        _drive(new Translation2d(0, 0), rotation);
        return this;
    }

    public DriveTrainSubsystem drive(Pose2d pose) {
        _drive(pose.getTranslation(), pose.getRotation().getRadians());
        return this;
    }

    public DriveTrainSubsystem drive(Pose3d pose) {
        drive(pose.toPose2d());
        return this;
    }

    public DriveTrainSubsystem drive() {
        _drive(new Translation2d(0, 0), 0);
        return this;
    }

    public void setJoystickAxes(TeleopSwerve.JoystickAxes axes) {
        
    }
}