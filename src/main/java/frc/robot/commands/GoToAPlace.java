package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.VisionSubsystem2024;

public class GoToAPlace extends Command {
    public static final int TAG = 1;
  private DriveTrainSubsystem drive;
  private PIDController xController, yController, thetaController;
  private double lastTime = 0;
  private Timer timer = new Timer();
//   private Pose2d initPose;
  private Pose2d targetPose;
  private VisionSubsystem2024 vis;
  private double targetDist;
//   private XboxController driver;
  private boolean robotCanDrive;
  private boolean isAuto;

  public GoToAPlace(DriveTrainSubsystem drive, VisionSubsystem2024 vision, Pose2d target, double dist) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPose = target;
    // this.driver = driver;
    vis = vision;  
    targetDist = dist;
    isAuto = true;
  }

  @Override
  public void initialize() {
    // initPose = drive.getPose();
    timer.reset();
    timer.start();
    
    xController = new PIDController(AutoConstants.kPAutoShoot, 0, 0);
    yController = new PIDController(AutoConstants.kPAutoShoot, 0, 0);
    thetaController = new PIDController(AutoConstants.kPTurnAutoShoot, 0, 0);
    thetaController.enableContinuousInput(0, Math.PI * 2D);
    // thetaController.setInputRange(Math.PI * 2);

    lastTime = 0;
    // vis.setDrivePosition(targetDist, "speakerTag");
    vis.setDrivePosition(targetDist, TAG);
    robotCanDrive = true;
  }

  @Override
  public void execute() {
    // TODO kill following line if it broken
    //vis.setDrivePosition(targetDist, "speakerTag");
    double time = timer.get();
    double dt = time - lastTime;
    Pose2d currentPose = drive.getPose();
    xController.setSetpoint(-targetPose.getX());
    yController.setSetpoint(-targetPose.getY());
    thetaController.setSetpoint(Math.toRadians(vis.getTheta(TAG)));

    // double vx = xController.calculate(-currentPose.getX(), dt) - refState.velocity.x;
    // double vy = yController.calculate(-currentPose.getY(), dt) - refState.velocity.y;
    // double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt) + refState.velocity.z;

    double vx = xController.calculate(currentPose.getX(), dt);
    double vy = yController.calculate(currentPose.getY(), dt);
    double omega = -thetaController.calculate(drive.getGyroHeading().getRadians(), dt);
  
    double cap = 1.5;
    int omegacap = 3;
    // Very dumb fix, should be x,y
    if(vx > cap) {
      vx = cap;
    } else if(vx < -cap) {
      vx = -cap;
    }
    if(vy > cap) {
      vy = cap;
    } else if(vy < -cap) {
      vy = -cap;
    }
    if(omega > omegacap ) {
      omega = omegacap;
    } else if(omega < -omegacap) {
      omega = -omegacap;
    }
    // System.out.println(vx);
    // System.out.println(vy);
    // System.out.println(omega);
    lastTime = time;
    if(Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2) < AutoConstants.autoShootCloseness && Math.abs(vis.getTheta(TAG) - drive.getGyroHeading().getDegrees()) < AutoConstants.degreesError) {
      robotCanDrive = false;
      System.out.println("HUH");
    }
    if(robotCanDrive) {
      drive.drive(new Translation2d(vx, vy), omega);
    } else {
      drive.drive(new Translation2d(0, 0), 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("lockon dead");
    timer.stop();
    drive.brake();
  }

  @Override    
  public boolean isFinished() {
    boolean retVal = false;
    if(isAuto) {
      if(timer.hasElapsed(2)) {
        
      }
      return timer.hasElapsed(2);
    }
    return retVal;
  }
}