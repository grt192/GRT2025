package frc.robot.subsystems.DiffySubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;  
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;


public class DiffyArmSubsystem {

    


    public DiffyArmSubsystem(int canId) {



    }

    //Default
    public void diffyDeafaltPos(){

        
    }




    //Wrist Position

    public void leftTwist(){

    }

    public void centerTwist(){
        
    }

    public void rightTwist(){

    }

    /* 
     * @param position the position of the wrist in degrees
     */
    public void setwristPosition(double position){
        
    }




    //Arm Position

    public void armUp(){

    }

    public void armMiddle(){

    }

    public void armDown(){

    }

    

    /*
     * @param position the position of the arm in degrees
     */
    public void setArmPosition(double position){
        
    }




    // Positions

    /*Get the position of both econders
     * @returns two ints, the first is left motor, the second is the right motor  
     */
    public double getDifferentialPosiution(){
        return 0;

    }


    /*
     * @returns the position of the arm in degrees
     */
    public double getDiffyArmPosition(){
        return 0;
    }

    /*
     * @returns the position of the wrist in degrees
     */
    public double getDiffyWristPosition(){
        return 0;
    }







}
