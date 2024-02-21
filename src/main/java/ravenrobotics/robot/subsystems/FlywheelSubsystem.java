package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    
    //Motors
    private final CANSparkMax topMotor = new CANSparkMax(FlywheelConstants.kTopFlyWheel, MotorType.kBrushless);
    private final CANSparkMax bottomMotor = new CANSparkMax(FlywheelConstants.kBottomFlyWheel, MotorType.kBrushless);
    
     public void configMotors()
    {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }
    
    public void shootOn() 
    {
        topMotor.set(1);
        bottomMotor.set(1);
    }

    public void stopFly()
    {
        topMotor.set(0);
        bottomMotor.set(0);
    }
}
