package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import static frc.robot.Constants.PivotConstants.pivotMotorID;


public class PivotSubsytem extends SubsystemBase{

    private SparkMax pivotMotor;
    private RelativeEncoder pivotEncoder; 
    private SparkClosedLoopController pivotPID;

    private double PivotP = 1; 
    private double PivotI = 0; 
    private double PivotD = 0;

    private double angle;

    public void PivotSubsystem(){
        
        pivotMotor = new SparkMax (pivotMotorID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getClosedLoopController();
    
    }

    public void setAngle(double angle){
        pivotPID.setReference(angle, ControlType.kPosition);
    }


    }

