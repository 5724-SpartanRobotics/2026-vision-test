package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Util.CTREModuleState;
import frc.robot.Util.Conversions;

public class SwerveModule {

    // Offset
    private double offset = 0 ;

    // Decleration of swerve components
    private TalonFX turn;
    /* Start at position 0, enable FOC, no feed forward, use slot 0 */
    private PositionVoltage turnVoltagePosition;
    private TalonFX drive;
    private CANcoder canCoder;

    // Desired speed and angle values for swerve
    private double driveSpeed = 0;//1 = max speed.
    private double driveAngle = 0;
    public String Name;
    private String canCoderName;
    private TalonFXConfiguration turnConfiguration;

    // Takes ID's of swerve components when called.
    public SwerveModule(int turnMotor, int driveMotor, int canCoderID, double off, String name, DriveTrainInterface driveTr) {
        Name = name;
        // Set the offset
        offset = off;

        // IDs for cancoder and falcons.
        turn = new TalonFX(turnMotor);
        turnVoltagePosition = new PositionVoltage(0);
        drive = new TalonFX(driveMotor);
        canCoder = new CANcoder(canCoderID);
        canCoderName = name + canCoderID;
        drive.getConfigurator().apply(new TalonFXConfiguration());
        drive.setNeutralMode(NeutralModeValue.Brake);
        turnConfiguration = new TalonFXConfiguration();
        turnConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        turnConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        turnConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        turnConfiguration.HardwareLimitSwitch.ReverseLimitEnable = false;
        turn.setNeutralMode(NeutralModeValue.Brake);
        // set slot 0 gains
        turnConfiguration.Slot0.kV = 0;
        turnConfiguration.Slot0.kP = 0.9;
        turnConfiguration.Slot0.kI = 0;
        turnConfiguration.Slot0.kD = 0;

        turnConfiguration.Voltage.PeakForwardVoltage = 10.0;
        turnConfiguration.Voltage.PeakReverseVoltage = -10.0;
        
        resetTurnToAbsolute();
        StatusCode status = turn.getConfigurator().apply(turnConfiguration);
        for (int i = 0; i < 5; ++i) {
          status = turn.getConfigurator().apply(turnConfiguration);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
        // Dalton Commented this out
        // turn.setPosition(0);
        turn.setControl(turnVoltagePosition.withPosition(0));

        // cancoder settings.
        // we used to set a coefficient to make the can coder feedback be in radians. The
        // new CTRE library seems to not let you do this, rather feedback is 0 - 1 in rotations.
        //config = new CANcoderConfiguration();
        //config.sensorCoefficient = 2 * 3.14 / 4096.0;
        //config.unitString = "rad";
        //config.sensorTimeBase = SensorTimeBase.PerSecond;
        //canCoder.configAllSettings(config);
    }

 
    /**
     * Sets up the turn motor configuration setting for FeedbackRotorOffset, but does not apply
     * configuration. Call turn.getConfigurator().apply after calling this.
     */
    private void resetTurnToAbsolute(){
        //get the absolute encoder position and subtract the starting offset, to be used to reset the encoder so it knows where we are
        double canCodePosInRadians = canCoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2.0);
        double absPosition = Conversions.radiansToFalcon(canCodePosInRadians - offset);
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + "Posn abs", absPosition);
        }
        //negate the position. The cancoder increases while the motor encoder decreases
        // Dalton added this and removed the line below
        turn.setPosition(-absPosition);
        //turnConfiguration.Feedback.FeedbackRotorOffset = -absPosition;
    }

    //Gets the current state of the robot based on the specified gyro angle and the last speed setpoint
    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(drive.getVelocity().getValueAsDouble());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(-turn.getPosition().getValueAsDouble()));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double position = Conversions.falconToMeters(drive.getPosition().getValueAsDouble());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(-turn.getPosition().getValueAsDouble()));
        return new SwerveModulePosition(position, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        driveSpeed = desiredState.speedMetersPerSecond / DriveConstants.maxRobotSpeedmps;
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " DriveRef", driveSpeed);
        }
        drive.set(driveSpeed);

        //if desired speed is less than 1 percent, keep the angle where it was to prevent jittering
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxRobotSpeedmps * 0.01)) ? driveAngle : desiredState.angle.getRadians();
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " TurnRef", Units.radiansToDegrees(angle));
        }
//        turn.setPosition(-Conversions.radiansToFalcon(angle));
        turn.setControl(turnVoltagePosition.withPosition(-Conversions.radiansToFalcon(angle)));
        driveAngle = angle;
    }

    public void periodic(){
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("Pos FB " + Name, Units.radiansToDegrees(Conversions.falconToRadians(turn.getPosition().getValueAsDouble())));
            SmartDashboard.putNumber(canCoderName,  Units.radiansToDegrees(canCoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2.0) - offset));
            SmartDashboard.putNumber("Drive FB " + Name, drive.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("DriveAngle " + Name, driveAngle);
        }
    }
  }