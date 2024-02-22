// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot.commands;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.IntakeSubsystem;
import ravenrobotics.robot.subsystems.MecDriveSubsystem;
import ravenrobotics.robot.subsystems.VisionSubsystem;
import ravenrobotics.robot.Constants.IntakeConstants.IntakeArmPosition;

public class VisionCommand extends Command {
  private final MecDriveSubsystem mecDriveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final String alliance;
   
  
  public VisionCommand(VisionSubsystem visionSubsystem, MecDriveSubsystem mecDriveSubsystem, IntakeSubsystem intakeSubsystem, Optional<Alliance> alliance) {
    
    //Initialize variables
    this.mecDriveSubsystem = mecDriveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.alliance = alliance.toString();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(visionSubsystem, mecDriveSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.resetValues();
    if(intakeSubsystem.isRetracted()==false){intakeSubsystem.setIntakePosition(IntakeArmPosition.kRetracted);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionSubsystem.updateTargetData();
    //Check that a valid target is in camera view and that it is a red or blue target
    
    if (!visionSubsystem.validTarget().toString().equals("NONE") && !visionSubsystem.getTColor().equals("none")){
      //if the target and alliance are both red or both blue
       if((visionSubsystem.getTColor().equals("red") && alliance.equals("red")) 
       || (visionSubsystem.getTColor().equals("blue") && alliance.equals("blue"))){
        //drive with values
        //mecDriveSubsystem.drive(m_strafe, m_forward, m_rotation, m_isFieldRelative);
        mecDriveSubsystem.drive(0, visionSubsystem.getForwardSpeed(), visionSubsystem.getRotationSpeed(), false);
       }
    }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return visionSubsystem.getTargetDistance()==visionSubsystem.getGoalRange();
  }
}
