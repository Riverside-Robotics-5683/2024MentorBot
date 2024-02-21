// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.MecDriveSubsystem;
import ravenrobotics.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
  /** Creates a new VisionCommand. */
  public VisionCommand(VisionSubsystem visionSubsystem, MecDriveSubsystem mecDriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(visionSubsystem, mecDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
