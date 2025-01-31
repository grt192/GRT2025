package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DebugConstants.CTRE_DEBUG;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Motors.LoggedTalon;


public class ElevatorSubsystem extends SubsystemBase{

    //motor stuff and configs
    private final LoggedTalon elevatorMotor;

    //limit switch
    private final DigitalInput zeroLimitSwitch;
    
    public ElevatorSubsystem() {
        zeroLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_ID); 
        elevatorMotor = new LoggedTalon(
            ElevatorConstants.KRAKEN_ID, ElevatorConstants.TALON_CONFIG
        );
    }

    @Override
    public void periodic() {
        if(atGround()) {
            elevatorMotor.setPosition(0);
        }
        elevatorMotor.logStats();
        if(CTRE_DEBUG || ElevatorConstants.DEBUG){
            elevatorMotor.publishStats();
        }
    }

    //functions
    public void setTargetState(ElevatorState targetState) {
        double targetPosition = targetState.getDistance();
        elevatorMotor.setPositionReference(targetPosition);
    }

    /**
     * Sends the elevator to the ground
     */
    public void toGround(){
        elevatorMotor.setPositionReferenceWithArbFF(
            ElevatorState.GROUND.getDistance(), ElevatorConstants.kArbFF
        );
    }

    public boolean atState(ElevatorState state) {
        double distance = Math.abs(this.getCurrentPosition() - state.getDistance());
        return distance < ElevatorConstants.TOLERANCE;
    }

    public double getCurrentPosition() {
        return elevatorMotor.getPosition();
    }

    public boolean atGround() {
        return zeroLimitSwitch.get();
    }
}