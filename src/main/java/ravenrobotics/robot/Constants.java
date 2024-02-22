package ravenrobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants 
{
    //Constants for controller IDs, etc.
    public static class DriverStationConstants
    {
        public static final int kDriverPort = 0;
        public static final int kSystemsPort = 1;
    }
    //Constants for the drivetrain, like motor IDs and whether they should be inverted.
    public static class DrivetrainConstants
    {
        /////////////////////
        //////Motor IDs//////
        /////////////////////
        public static final int kFrontLeftMotor = 2;
        public static final int kFrontRightMotor = 3;
        public static final int kBackLeftMotor = 4;
        public static final int kBackRightMotor = 5;
        /////////////////////
        ////Invert Motors////
        ////////////////////
        public static final boolean kInvertFrontLeftSide = true;
        public static final boolean kInvertFrontRightSide = false;
        public static final boolean kInvertBackLeftSide = true;
        public static final boolean kInvertBackRightSide = false;
        /////////////////////
        /////Max Voltage/////
        /////////////////////
        public static final int kDriveMaxVoltage = 12;
        public static final double kDriveSysIDVoltageRampRate = 0.5;
        /////////////////////
        ///Other Constants///
        /////////////////////
        //TODO: Find actual speed of the drivetrain, using number from online for now.
        public static final double kDriveMaxSpeedMPS = Units.feetToMeters(13.87);
        //Slew rates.
        public static final double kTranslationSlewRate = 1.5;
        public static final double kRotationSlewRate = 1.0;
        /////////////////////
        //Encoder Constants//
        /////////////////////
        public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
        public static final double kEncoderConversionFactor = Math.PI * kWheelDiameterMeters;
    }
    //Constants for the Pigeon2 IMU, such as the ID and various configuration settings.
    public static class IMUConstants
    {
        //ID of the Pigeon2.
        public static final int kPigeon2ID = 1;
        public static final String kPigeon2CANBus = "ctre";
        //Whether the IMU should default to future (potentially unsupported) configs.
        public static final boolean kFutureProofConfigs = false;
        //IMU trim for readings.
        public static final double kTrimX = 0.0;
        public static final double kTrimY = 0.0;
        public static final double kTrimZ = 0.0;
        //Mounting position in degrees.
        public static final double kMountPitch = 0.0;
        public static final double kMountRoll = 0.0;
        public static final double kMountYaw = 0.0;
        //Whether specific features should be enabled. (keep them to false)
        public static final boolean kDisableNoMotionCalibration = false;
        public static final boolean kDisableTemperatureCompensation = false;
    }
    //Constants for the intake.
    public static class IntakeConstants
    {
        //Motor constants.
        public static final int kRollerMotor = 6;
        public static final int kArmMotor = 7;
        //PID Constants for the arm.
        public static final double kArmP = 1.5;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.5;
        
        public static final double kArmDeployedSetpoint = 57;

        public static final double kRollerFF = 1.0;
        public static final double kRollerSetpoint = 4000;
        public static final double kNoteInDistance = 6;
        //enums for deployed and retracted arm position
        public static enum IntakeArmPosition {kDeployed,kRetracted}
    }

    //Constants for the flywheel.
    public static class FlywheelConstants
    {
        //Fly Wheel Motor Id's
        public static final int kTopFlyWheel = 8;
        public static final int kBottomFlyWheel = 9;
        //Inverting Fly Wheel
        public static final boolean kInvertTopFlyWheel = false;
        public static final boolean kInvertBottomFlyWheel = true;
        //Setpoint
        public static final double kSetPoint = 3000;
    }

    //Constants for kinematics.
    public static class KinematicsConstants
    {
        //TODO: Check this are kTrackWidth and kWheelBase the same measurement as the offset?
        
        /*
        //Example code...
        //Offset from the center of the robot to a wheel.
        public static final double kTrackWidth = 0.5;
      
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between centers of front and back wheels on robot
    
        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        */
        public static final double kOffset = Units.inchesToMeters(30) / 2;
        //Translation2d offsets for each wheel.
        public static final Translation2d kFrontLeftOffset = new Translation2d(kOffset, kOffset);
        public static final Translation2d kFrontRightOffset = new Translation2d(kOffset, -kOffset);
        public static final Translation2d kBackLeftOffset = new Translation2d(-kOffset, kOffset);
        public static final Translation2d kBackRightOffset = new Translation2d(-kOffset, -kOffset);
        //Actual kinematics object for performing calculations.
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
            kFrontLeftOffset,
            kFrontRightOffset,
            kBackLeftOffset,
            kBackRightOffset);
    }
    public static class VisionConstants{
        public static final String kCameraName = "ravenVision";
        //Constants such as camera and target height stored. Change per robot and goal!
        //TODO: Should target height be offset to top or center? Target is 6.5 square. 

        //Current heights are to bottom edge.Stage: 48.88, Amp 50.13, Speaker 53.88, Source 4.125
        public static final double kCameraHeight = Units.inchesToMeters(24);
        public static final double kTargetHeightAmp = Units.feetToMeters(50.13);
        public static final double kTargetHeightSpeaker = Units.feetToMeters(53.88);

        //TODO: Do we want to line up on source or stage?
        
        //public static final double kTargetHeightStage = Units.feetToMeters(48.88); 
        //public static final double kTaargetSource = Units.feetToMeters(4.125);

        // Angle between horizontal and the camera.
        public static final double kCameraPitchAngle = Units.degreesToRadians(0);

        // How far from the targets we want to be
        public static final double kGoalRangeAmp = Units.feetToMeters(3);
        public static final double kGoalRangeSpeaker = Units.feetToMeters(3);

        // AprilTagIds

        //Right and Left from Driver's viewpoint
        /////BLUE ID'S/////
        public static final int kBlueAmpID = 6;
        public static final int kBlueCSpeakerID = 7;
        //public static final int kBlueRSrcID = 1;
        //public static final int kBlueLSrcID = 2;
        //public static final int kBlueRSpeakerID = 8;
        //public static final int kBlueStageCID = 14;
        //public static final int kBlueStageRID = 16;
        //public static final int kBlueStageLID = 15;
          
        /////RED ID'S/////
        public static final int kRedCSpeakerID = 4;
        public static final int kRedAmpID = 5;
        
        //public static final int kRedRSpeakerID = 3;
        //public static final int kRedRSrcID = 9;
        //public static final int kRedLSrcID = 10;
        //public static final int kRedStageCID = 13;
        //public static final int kRedStageRID = 12;
        //public static final int kRedStageLID = 14;
      
        //TODO: Update check to include more vision targets if needed.
        public static enum ValidTarget{NONE, BLUE_AMP, RED_AMP, BLUE_SPEAKER, RED_SPEAKER} 
    }
}
