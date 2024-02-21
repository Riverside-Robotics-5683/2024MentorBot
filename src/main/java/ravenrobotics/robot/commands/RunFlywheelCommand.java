package ravenrobotics.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.FlywheelSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem;

public class RunFlywheelCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    
    public RunFlywheelCommand(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        
        //Add the subsystem as a requirement for the command, so the subsystem isn't being controlled by two different commands at once.
        addRequirements(intakeSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize()
    {
        flywheelSubsystem.configMotors();
    }

    @Override
    public void execute()
    {
        flywheelSubsystem.shootOn();
        Timer.delay(1.2);
        intakeSubsystem.runRollers();
    }

    @Override
    public void end(boolean interrupted)
    {
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
    }
}
