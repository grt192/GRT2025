package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.swerve.KrakenDriveMotor;

public class ElevatorSubsystem extends SubsystemBase{

    //motor stuff
    private final TalonFX elevatorMotor;
    private PositionVoltage request;
    private Slot1Configs slot1Configs;

    //states
    private ElevatorState state = ElevatorState.ZERO_STATE;
    private ElevatorState targetState = ElevatorState.ZERO_STATE;

    //limit switch
    private final DigitalInput zeroLimitSwitch;
    
    public ElevatorSubsystem() {

        zeroLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_ID); 
        elevatorMotor = new TalonFX(0);

        request = new PositionVoltage(0); //change
    }

    //pid stuff
    slot1Configs = new Slot1Configs();
    slot1Configs.kP = ElevatorConstants.ELEVATOR_P;
    slot1Configs.kI = ElevatorConstants.ELEVATOR_I;
    slot1Configs.kD = ElevatorConstants.ELEVATOR_D;

    elevatorMotor.getConfigurator().apply(slot1Configs);

    //functions
    public void setTargetState(ElevatorState targetState) {
        elevatorMotor.setControl(request.withPosition(null)); //change
        this.targetState = targetState;
    }

    public ElevatorState getCurrentState() {
        return this.targetState;
    }

    public boolean atState(ElevatorState state) {
        double distance = Math.abs(this.getCurrentPosition() - state.getExtendDistanceMeters());
        return distance < ElevatorConstants.TOLERANCE;
    }

    public double getCurrentPosition() { 
        return elevatorMotor.getPosition(); //change from angles to position
    }

    public boolean atGround() {
        double distance = Math.abs(this.getCurrentPosition() - state.getExtendDistanceMeters());
        return distance < ElevatorConstants.TOLERANCE && zeroLimitSwitch.get();
    }

    public void periodic() {

        if(atGround()) {
            elevatorMotor.setPosition(0);
        }

    }



}