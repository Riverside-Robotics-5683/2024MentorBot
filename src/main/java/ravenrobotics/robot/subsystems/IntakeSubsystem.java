package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Roller motor and encoder.
    private final CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotor, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    
    //Arm motor and encoder.
    private final CANSparkMax armMotor = new CANSparkMax(IntakeConstants.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder armMotorEncoder = armMotor.getEncoder();

    //PID Controller for the arm.
    private final SparkPIDController armPIDController = armMotor.getPIDController();
    
    //TODO: Known issue with Onboard I2C port. Use MXP port?
    
    //Distance sensor.
    private final Rev2mDistanceSensor distanceSensorMXP = new Rev2mDistanceSensor(Port.kMXP);
    //private final Rev2mDistanceSensor distanceSensorOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    
    //enums for deployed and retracted arm position
    public enum IntakeArmPosition {kDeployed,kRetracted}

    //Subsystem for controlling the intake on the robot.
    public IntakeSubsystem()
    {
        //Configure the motors, encoders and subsystem for use.
        configMotors();
        configEncoders();
        distanceSensorMXP.setRangeProfile(RangeProfile.kHighSpeed);
        distanceSensorMXP.setAutomaticMode(true);
    }

    /**
     * Sets the intake position.
     * 
     * @param position The desired position of the intake.
     */
    public void setIntakePosition(IntakeArmPosition position)
    {
        switch(position)
        {
            case kDeployed -> armPIDController.setReference(IntakeConstants.kArmDeployedSetpoint, ControlType.kPosition);
            case kRetracted -> armPIDController.setReference(0, ControlType.kPosition);
        }
    }

    //push game piece out with rollers
    public void runRollers(){rollerMotor.set(1);}

    //pull game piece in with rollers
    public void intakeRunRollers(){rollerMotor.set(-1);}

    //check if game piece is loaded
    public boolean isLoaded(){
        double noteDistance = getNoteDistance();
        if (noteDistance < IntakeConstants.kNoteInDistance){return true;}
        else {return false;}       
    }

    //TODO: Test Arm Position when retracted
    //check if arm is retracted
    public boolean isRetracted(){
        if(armMotorEncoder.getPosition() > 0.5) {return false;}
        else{return true;}
    }

    //TODO: Check sensor values. Will return 99.0 if range is not valid. 
    //get note distance to be used in intake routine.
    public double getNoteDistance(){
    if(distanceSensorMXP.isRangeValid()) {return distanceSensorMXP.getRange(Unit.kInches);}
    else {return 99.0;}
    }
    
    public void stopRollers() {rollerMotor.stopMotor();}

    /**
     * Interrupts (cancels) the roller thread.
     */
  //  public void cancelWaitRoutine(){ rollerCommand.cancel();}

    @Override
    public void periodic()
    {
        System.out.println("Arm Position: " + armMotorEncoder.getPosition());
        System.out.println("Note Distance: " + distanceSensorMXP.getRange(Unit.kInches));
    }

    /**
     * Configures the subsystem's motors for use.
     */
    private void configMotors()
    {
        //Reset to factory defaults.
        rollerMotor.restoreFactoryDefaults();
        
        //Set idle mode.
        rollerMotor.setIdleMode(IdleMode.kCoast);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.setClosedLoopRampRate(0.01);
        armPIDController.setFeedbackDevice(armMotorEncoder);
        armPIDController.setOutputRange(-0.25, 0.5);

        //Set the PID constants for the PID controller.
        armPIDController.setP(IntakeConstants.kArmP);
        armPIDController.setI(IntakeConstants.kArmI);
        armPIDController.setD(IntakeConstants.kArmD);

        //Save configuration.
        rollerMotor.burnFlash();
        armMotor.burnFlash();
    }

    /**
     * Configures the subsystem's encoders for use.
     */
    private void configEncoders()
    {
        rollerMotorEncoder.setPosition(0.0);
        armMotorEncoder.setPosition(0.0);
    }
}
