// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.IntakeSubsystem;
import ravenrobotics.robot.subsystems.MecDriveSubsystem;
import ravenrobotics.robot.subsystems.VisionSubsystem;
import ravenrobotics.robot.Constants.IntakeConstants.IntakeArmPosition;

public class VisionCommand extends Command {
  private final MecDriveSubsystem mecDriveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Optional<Alliance> alliance;
   
  
  public VisionCommand(VisionSubsystem visionSubsystem, MecDriveSubsystem mecDriveSubsystem, IntakeSubsystem intakeSubsystem) {
    
    //Initialize variables
    this.mecDriveSubsystem = mecDriveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.alliance = DriverStation.getAlliance();
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
      if(alliance.isPresent()){
      //if the target and alliance are both red or both blue turn and approach
       if((visionSubsystem.getTColor().equals("red") && alliance.get()==Alliance.Red) 
       || (visionSubsystem.getTColor().equals("blue") && alliance.get()==Alliance.Red))
       {
        //drive with values
        //mecDriveSubsystem.drive(m_forward, m_strafe, m_rotation, m_isFieldRelative);
        mecDriveSubsystem.drive(visionSubsystem.getForwardSpeed(), 0, visionSubsystem.getRotationSpeed(), false);
       }
      }
      else{
        //else turn but don't approach target.
        //mecDriveSubsystem.drive(m_forward, m_strafe, m_rotation, m_isFieldRelative);
        mecDriveSubsystem.drive(0, 0, visionSubsystem.getRotationSpeed(), false);
        
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
