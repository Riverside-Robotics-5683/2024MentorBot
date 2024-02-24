// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

//import java.util.OptionalInt;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DriverStationConstants;
import ravenrobotics.robot.commands.MecDriveCommand;
import ravenrobotics.robot.commands.RunFlywheelCommand;
import ravenrobotics.robot.commands.RunIntakeCommand;
import ravenrobotics.robot.commands.VisionCommand;
import ravenrobotics.robot.subsystems.*;
import ravenrobotics.robot.Constants.IntakeConstants.IntakeArmPosition;
import ravenrobotics.robot.util.Telemetry;

public class RobotContainer 
{
  // The robot's subsystems
  private final MecDriveSubsystem mecDriveSubsystem = new MecDriveSubsystem();
  private final FlywheelSubsystem flyWheelSubsystem = new FlywheelSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  //Driver controller (drives the robot around).
  private final CommandJoystick driverJoystick = new CommandJoystick(DriverStationConstants.kDriverPort);
  
  //TODO: MAP CONTROLLERS
  //Systems controller (runs intake, shooter, climber)
  //private final CommandXboxController systemsController = new CommandXboxController(DriverStationConstants.kSystemsPort);

  //Whether to drive field relative or not.
  public boolean isFieldRelative = false;

  private GenericEntry isFieldRelativeEntry = Telemetry.teleopTab.add("Field Relative", false).getEntry();

  private final SendableChooser<Command> teleopModeChooser = new SendableChooser<Command>();

  //Main drive command.
  //TODO check controller getY is forward right?
  private final MecDriveCommand mecDriveCommand = new MecDriveCommand(   
    () -> -driverJoystick.getY(),
    () -> -driverJoystick.getX(),
    () -> -driverJoystick.getZ(),
    () -> -driverJoystick.getThrottle(),
    () -> isFieldRelative, 
    mecDriveSubsystem);

  public RobotContainer()
  {
    //Add drive command to TeleOp mode chooser.
    teleopModeChooser.addOption("Drive Mec", mecDriveCommand);

    //Put the TeleOp mode chooser on the dashboard.
    Telemetry.teleopTab.add("TeleOp Mode", teleopModeChooser);
    
    //Configure configured controller bindings.
    configureBindings();
    mecDriveSubsystem.setDefaultCommand(mecDriveCommand);
  }
  
  private void configureBindings()
  {
    // Shooting: parallel command to run intake rollers and flywheel while trigger held 
    driverJoystick.button(1).whileTrue(new RunFlywheelCommand(intakeSubsystem, flyWheelSubsystem));
  
    //Set the buttons on the joystick to toggle field-relative
    driverJoystick.button(2).onTrue(new InstantCommand(() -> toggleFieldRelative()));

    //TODO: Set the button on the joystick for aligning to AprilTag and driving to range
    driverJoystick.button(3).whileTrue(new VisionCommand(visionSubsystem, mecDriveSubsystem,intakeSubsystem));
    
    //Run Intake Command 
    driverJoystick.button(4).toggleOnTrue(new RunIntakeCommand(intakeSubsystem));

    //Set the buttons on the joystick for deploying and retracting the arm
    driverJoystick.button(5).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakePosition(IntakeArmPosition.kDeployed), intakeSubsystem));
    driverJoystick.button(6).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakePosition(IntakeArmPosition.kRetracted), intakeSubsystem));
 
    //TODO: Use case for ZeroYaw on button press? Testing?
    driverJoystick.button(7).onTrue(new InstantCommand(() -> IMUSubsystem.zeroYaw()));
    
    //Set the buttons on the joystick for running intake rollers
    driverJoystick.button(11).toggleOnTrue(new InstantCommand(() -> intakeSubsystem.intakeRunRollers()));
    //TODO: Use case for stopping intake rollers? Testing?
    driverJoystick.button(12).toggleOnFalse(new InstantCommand(() -> intakeSubsystem.stopRollers()));
  }

  public void setupTeleopCommand()
  {
    Command selectedCommand = teleopModeChooser.getSelected();
    if (selectedCommand == null)
    {
      mecDriveSubsystem.setDefaultCommand(mecDriveCommand);
      return;
    }
    mecDriveSubsystem.setDefaultCommand(selectedCommand);
  }

  private void toggleFieldRelative()
  {
    //Toggle field relative (if true set false, if false set true)
    if (isFieldRelative)
    {
      isFieldRelative = false;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
    else
    {
      isFieldRelative = true;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
