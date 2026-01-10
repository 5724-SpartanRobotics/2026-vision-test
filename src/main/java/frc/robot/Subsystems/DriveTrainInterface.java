package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/*
An interface implemented by the drive train
*/
public interface DriveTrainInterface {
    Rotation2d getGyroHeading();
    DriveTrainSubsystem drive(Translation2d translation, double rotation);
}
