package frc.robot.Util;

import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.DriveConstants;

public class Conversions {
    /**
     * Converts falcon motor counts to radians - specific to the turn motor
     * @param counts The counts to be converted to radians
     * @return The radians for the specified counts.
     */
    public static double falconToRadians(double counts){
        return counts * (Constant.TwoPI / (DriveConstants.turnGearRatio * 1.0));
    }

    /**
     * Converts turn motor sensor counts to wheel degrees using the turn gear ratio from DriveConstants
     * @param counts Falcon sensor counts
     * @return Wheel angle in degrees
     */
    public static double falconToDegrees(double counts){
        return counts * (360.0 / (DriveConstants.turnGearRatio * 1.0));
    }

    /**
     * Converts radians to Falcon motor counts. Specific to the turn motor
     * @param radians the radians to convert to Falcon motor counts
     * @return Falcon Counts - 1.0 = 1 rotation of the motor
     */
    public static double radiansToFalcon(double radians){
        return radians / (Constant.TwoPI / (DriveConstants.turnGearRatio * 1.0));
    }

    public static double degreesToFalcon(double degrees){
        return degrees / (360 / (DriveConstants.turnGearRatio * 1.0));
    }
    /**
     * Converts the drive motor velocity counts to motor RPM using the DriveConstants driveGearRatio
     * @param velocitycounts Falcon motor counts which are counts in the last .1 seconds
     * @return Motor RPM
     */
    public static double falconToRPM(double velocitycounts)
    {
        //motor controller velocity counts are counts in 100 ms, or 0.1 seconds.
        // in one minute there are 600 of these .1 second intervals
        double motorRPM = velocitycounts * (600.0 / 1.0);
        double wheelRPM = motorRPM / DriveConstants.driveGearRatio;
        return wheelRPM;
    }

    /**
     * Converts drive motor velocity counts to wheel MPS
     * @param velocitycounts
     * @return
     */
    public static double falconToMPS(double velocitycounts)
    {
        double wheelRPM = falconToRPM(velocitycounts);
        double wheelMPS = (wheelRPM * DriveConstants.wheelCircumfrence)/60;
        return wheelMPS;
    }

    public static double falconToMeters(double positionCounts)
    {
        double distanceMeters = positionCounts / 1.0 / DriveConstants.driveGearRatio * DriveConstants.wheelCircumfrence;
        return distanceMeters;
    }

    ///
    /**
     * Converts drive motor RPM to m/s.
     * @param rpm
     * @return
     */
    public static double rpmToMps(double rpm) {
        return rpm * Constant.TwoPI * Units.inchesToMeters(DriveConstants.wheelDiameter) / 60;
    }

    public static Integer[] intArrayToIntegerArray(int[] intArray) {
        Integer[] iArray = new Integer[intArray.length];
        for (int i = 0; i < intArray.length; i++) {
            iArray[i] = intArray[i];
        }
        return iArray;
    }
}
