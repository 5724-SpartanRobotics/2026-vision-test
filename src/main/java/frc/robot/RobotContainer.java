package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem2024;
import frc.robot.commands.GoToAPlace;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {
    private final DriveTrainSubsystem drive;
    // private final PhotonVisionSubsystem photon;
    private final VisionSubsystem2024 vision;
    // private final ApriltagLockon2Subsystem vision;
    // private final ApriltagLockonSubsystem vision;
    // private final LimelightLockonSubsystem limelightLockon;
    private final CommandJoystick drivestick;

    private JoystickButton jb_ZeroGyro;
    private JoystickButton jb_AlignApriltag;
    private JoystickButton jb_AlignLimelighObject;
    private JoystickButton jb_Rotate180;

    public RobotContainer() {
        this.drive = new DriveTrainSubsystem();
        // this.photon = new PhotonVisionSubsystem("Front");
        this.drivestick = new CommandJoystick(0);

        this.vision = new VisionSubsystem2024(drive);
        // this.vision = new ApriltagLockon2Subsystem(drive, photon);
        // this.vision = new ApriltagLockonSubsystem(drive, photon, drivestick);
        // this.limelightLockon = new LimelightLockonSubsystem(drive);

        // this.jb_ZeroGyro = new JoystickButton(drivestick, );
        // this.jb_AlignApriltag = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.TagLockonAlt);
        // this.jb_AlignLimelighObject = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.ObjectLockonAlt);
        // this.jb_Rotate180 = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.Rotate180);

        configureButtonBindings();
        
        drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));
    }
        
    private void configureButtonBindings() {
        drivestick.button(Constant.ControllerConstants.ButtonMap.TagLockonAlt).whileTrue(
            // new GotToAPlace2024(drive, vision, new Pose2d(), 3, drivestick, false)
            // new ApriltagLockon2Command(drive, vision, drivestick, new int[] {1}, 3)
            // new ApriltagAlignToTargetCommand(drive, vision, new Pose2d(), 1, drivestick, false)
            // new AprilTagLimelight(drive, limelightLockon, drivestick)
            // new AprilTagPhoton(photon.getCamera(), drive, drive.getPoseEstimator())
            new GoToAPlace(drive, this.vision, new Pose2d(1.,1., new Rotation2d(0.,0.)), 3.)
            
        );
        // jb_AlignLimelighObject.whileTrue(
        //     new LimelightAlignToTargetCommand(drive, limelightLockon, drivestick, 1, false)
        // );
        drivestick.button(Constant.ControllerConstants.ButtonMap.GyroZero).onTrue(new InstantCommand(() -> {drive.setGyroZero();}));
        drivestick.button(Constant.ControllerConstants.ButtonMap.Rotate180).onTrue(new InstantCommand(() -> {drive.drive(-Math.PI);}));
    }
}
