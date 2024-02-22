package ravenrobotics.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.IntakeSubsystem;
import ravenrobotics.robot.Constants.IntakeConstants;

public class RunIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    
    public RunIntakeCommand(IntakeSubsystem intakeSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        //Add the subsystem as a requirement for the command, so the subsystem isn't being controlled by two different commands at once.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        //deploy arm if retracted
        if(intakeSubsystem.isRetracted()){intakeSubsystem.setIntakePosition(IntakeConstants.IntakeArmPosition.kDeployed);}
    }

    @Override
    public void execute() {
        //run rollers to intake game piece
        intakeSubsystem.intakeRunRollers();
    }

    @Override
    public boolean isFinished() {
        //end if the gamepiece is loaded
        return intakeSubsystem.isLoaded();
    }

    @Override
    public void end(boolean interrupted)
    {
        //stop rollers and retract arm when intake routine is complete
        intakeSubsystem.stopRollers();
        intakeSubsystem.setIntakePosition(IntakeConstants.IntakeArmPosition.kRetracted);
    }
}
