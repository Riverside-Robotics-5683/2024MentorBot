// Modified WpiLib MecDrive for CANSparkMax motors and using IMU Subsystem static calls for gyro.

package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;

public class MecDriveSubsystem extends SubsystemBase {
 //Drivetrain motors. Configured for use with a NEO motor, which is brushless.
    private final CANSparkMax m_frontLeft = new CANSparkMax(DrivetrainConstants.kFrontLeftMotor, MotorType.kBrushless);
    private final CANSparkMax m_frontRight = new CANSparkMax(DrivetrainConstants.kFrontRightMotor, MotorType.kBrushless);
    private final CANSparkMax m_rearLeft = new CANSparkMax(DrivetrainConstants.kBackLeftMotor, MotorType.kBrushless);
    private final CANSparkMax m_rearRight = new CANSparkMax(DrivetrainConstants.kBackRightMotor, MotorType.kBrushless);

  //Mecanum Drive
    private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  //Drivetrain encoders.
    private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
    private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
    private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
    private final RelativeEncoder m_rearRightEncoder  = m_rearRight.getEncoder();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
            KinematicsConstants.kDriveKinematics,
            IMUSubsystem.getRotation2d(), 
            new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public MecDriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    //Restore all the motors to factory defaults, so that we can start fresh and nothing interferes.
    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();
    
    //Sets the velocity conversion factor to give us m/s.
    m_frontLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_rearLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_rearRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
     
    //Sets the position factor to give us accurate distance measurements.
    m_frontLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_rearLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    m_rearRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
 
    //TODO: Check Inversion s/b Left or Right
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 

    // Reverse the default direction of the left side so everything drives normally.
    m_frontLeft.setInverted(DrivetrainConstants.kInvertFrontLeftSide);//true
    m_frontRight.setInverted(DrivetrainConstants.kInvertFrontRightSide);//false
    m_rearLeft.setInverted(DrivetrainConstants.kInvertBackLeftSide);//true
    m_rearRight.setInverted(DrivetrainConstants.kInvertBackRightSide);//false
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(IMUSubsystem.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(IMUSubsystem.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and angular speeds. Speeds range from [-1, 1]
   * @param strafe Speed of the robot in the x direction (sideways).
   * @param forward Speed of the robot in the y direction (forward/backwards).
   * @param rotation Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(strafe, forward, rotation, IMUSubsystem.getRotation2d());
    } else {
      m_drive.driveCartesian(strafe, forward, rotation);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
  //Set positions to 0.
    m_frontLeftEncoder.setPosition(0.0);
    m_frontRightEncoder.setPosition(0.0);
    m_rearLeftEncoder.setPosition(0.0);
    m_rearRightEncoder.setPosition(0.0);
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(Double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    IMUSubsystem.zeroYaw();;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return IMUSubsystem.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -IMUSubsystem.getRate();
  }

  public void stopDrive(){
    m_frontLeft.stopMotor();
    m_rearLeft.stopMotor();
    m_frontRight.stopMotor();
    m_rearRight.stopMotor();
  }
}

