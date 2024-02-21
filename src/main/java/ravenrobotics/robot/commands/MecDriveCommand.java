// Modified Mecanum drive command class

package ravenrobotics.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.MecDriveSubsystem;
import ravenrobotics.robot.util.Telemetry;

public class MecDriveCommand extends Command {

  //Suppliers for joystick values and whether to drive field relative.
  private final DoubleSupplier strafe, forward, rotation, throttle;
  private final Boolean isFieldRelative;
  private final MecDriveSubsystem mecDriveSubsystem;
  /**
 * Command to drive the robot using joystick axes.
 * 
 * @param strafe The x axis for moving left/right.
 * @param forward The y axis for moving forward/backward.
 * @param rotation The z axis for rotating.
 * @param throttle The axis for controlling the max speed of the robot.
 * @param isFieldRelative The boolean for whether to drive field relative.
 */

  /** Creates a new MecDriveCommand. */
  public MecDriveCommand(DoubleSupplier strafe, 
    DoubleSupplier forward, DoubleSupplier twist, 
    DoubleSupplier throttle, BooleanSupplier isFieldRelative, 
    MecDriveSubsystem mecDriveSubsystem){

    //Initialize subsystem
    this.mecDriveSubsystem = mecDriveSubsystem;
    
    //Initialize boolean supplier (whether to drive field or robot relative)
    this.isFieldRelative = isFieldRelative.getAsBoolean();
    this.strafe = strafe;
    this.forward = forward;
    this.rotation = twist;
    this.throttle = throttle;
  
    //Add the subsystem as a requirement for the command, 
    //so the subsystem isn't being controlled by two different commands at once.
    addRequirements(mecDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Telemetry.switchToTeleopTab();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Temp variables
    Double maxSpeed, m_strafe, m_forward, m_rotation;
    Boolean m_isFieldRelative;

    maxSpeed = this.throttle.getAsDouble();
    m_strafe = this.strafe.getAsDouble();
    m_forward = this.forward.getAsDouble();
    m_rotation = this.rotation.getAsDouble();
    m_isFieldRelative = this.isFieldRelative.booleanValue();

    //Convert the thottle axis (mSpeed) to a range of 0-1.
    maxSpeed = (maxSpeed + 1) * 0.5;
    if (maxSpeed  < 0.1) { maxSpeed  = 0.1;}
    
    //set throttle as max output
    mecDriveSubsystem.setMaxOutput(maxSpeed);

    //TODO: Make these values constants for easy adjustment.
    //Account for controller deadband
    if (Math.abs(m_strafe) < 0.03){m_strafe = 0.0;}
    if (Math.abs(m_forward) < 0.03){m_forward = 0.0;}
    if (Math.abs(m_rotation) < 0.01){m_rotation = 0.0;}
    
    //drive with values
    mecDriveSubsystem.drive(m_strafe, m_forward, m_rotation, m_isFieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mecDriveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
