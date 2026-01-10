package frc.robot.Subsystems;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem2024 extends SubsystemBase {
    public static DoubleArraySubscriber _aprilTag;
    private static NetworkTableInstance _NetworkTable = NetworkTableInstance.getDefault();
    private static double[] _defaultTag = { -1, -1 };
    private DriveTrainSubsystem _DriveTrainSubsystem;

    public VisionSubsystem2024(DriveTrainSubsystem driveTrainSubsystem) {
        this._DriveTrainSubsystem = driveTrainSubsystem;
    }

    public void setDrivePosition(double targetDistance, int tag) {
        Optional<TagMap> optTag = TagMap.getTagMap(tag);
        if (optTag.isEmpty()) return;
        TagMap tagMap = optTag.get();
        // [ tag_dist_ft, tag_angle_deg, tag_id, tag_is_valid ]
        double[] reportedTag = tagMap.getSubscriber().get(_defaultTag);

        if (reportedTag.length == 4 && reportedTag[0] > 0) {
            double theta = -reportedTag[1] + _DriveTrainSubsystem.getGyroHeading().getDegrees() - Constants.CameraAngleOffset;
            double thetaRadians = Math.toRadians(theta);
            double deltaD = (reportedTag[0] - targetDistance + Constants.CameraDepthOffset) / 3.28;
            double deltaX = -deltaD * Math.sin(thetaRadians);
            double deltaY = deltaD * Math.cos(thetaRadians);
            _DriveTrainSubsystem.ZeroDriveSensors(makePose(deltaX, deltaY));
            tagMap.setLastTheta(theta);
        } else {
            _DriveTrainSubsystem.ZeroDriveSensors(makePose(0., 0.));
        }
    }

    public Pose2d makePose(double raw_dx, double raw_dy) {
        return new Pose2d(new Translation2d(-raw_dy, raw_dx), _DriveTrainSubsystem.getGyroHeading().times(-1.0));
    }

    public double getTheta(int tag) {
        Optional<TagMap> optTag = TagMap.getTagMap(tag);
        if (optTag.isEmpty()) return 0.0;
        return optTag.get().getLastTheta();
    }

    public boolean hasTag(int tag) {
        Optional<TagMap> optTag = TagMap.getTagMap(tag);
        if (optTag.isEmpty()) return false;
        TagMap tagMap = optTag.get();
        double[] reportedTag = tagMap.getSubscriber().get(_defaultTag);
        if (reportedTag.length != 4) return false;
        return reportedTag[0] > 0;
    }

    public enum TagMap {
        TAG0_InsertName( 0, _NetworkTable.getDoubleArrayTopic("/Vision/aprilTag0").subscribe(_defaultTag), 0. ),
        TAG1_InsertName( 1, _NetworkTable.getDoubleArrayTopic("/Vision/aprilTag1").subscribe(_defaultTag), 0. ),
        TAG2_InsertName( 2, _NetworkTable.getDoubleArrayTopic("/Vision/aprilTag2").subscribe(_defaultTag), 0. );

        private int _id;
        private DoubleArraySubscriber _subscriber;
        private double _lastTheta;
        TagMap(int id, DoubleArraySubscriber subscriber, double lastTheta) {
            this._id = id;
            this._subscriber = subscriber;
            this._lastTheta = lastTheta;
        }

        int getId() { return this._id; }
        DoubleArraySubscriber getSubscriber() { return this._subscriber; }

        double setLastTheta(double theta) { this._lastTheta = theta; return theta; }
        double getLastTheta() { return this._lastTheta; }

        static Optional<TagMap> getTagMap(int id) {
            return Arrays.asList(TagMap.values()).stream().filter(tm -> tm.getId() == id).findFirst();
        }
    }

    public static final class Constants {
        // TODO: I ned to fix this
        static final int CameraAngleOffset = 40;
        static final int CameraDepthOffset = 3;
    }
}
