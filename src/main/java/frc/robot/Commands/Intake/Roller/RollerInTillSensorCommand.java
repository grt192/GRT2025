package frc.robot.Commands.Intake.Roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstans.RollerConstants;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

public class RollerInTillSensorCommand extends Command{
    
    private RollerSubsystem rollerSubsystem;

    public RollerInTillSensorCommand(RollerSubsystem rollerSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        this.addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize(){
        rollerSubsystem.setDutyCycle(-RollerConstants.ROLLER_DUTY_CYCLE_IN_SPEED);
    }

    @Override
    public void end(boolean interrupted){
        rollerSubsystem.setDutyCycle(0);
    }

    @Override
    public boolean isFinished(){
        return rollerSubsystem.getIntakeSensor();
    }
}
