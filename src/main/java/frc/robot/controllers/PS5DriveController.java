package frc.robot.controllers;

import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PS5DriveController extends BaseDriveController{

    private final PS5Controller driveController = new PS5Controller(0);
    private double deadZone = 0;
    
    @Override
    public double getForwardPower() {
        double forwardPower = -driveController.getLeftY();
        if(Math.abs(forwardPower) > deadZone)
            return -driveController.getLeftY();
        else 
            return 0;
    }

    @Override
    public double getLeftPower(){
        double leftPower = -driveController.getLeftX();
        if(Math.abs(leftPower) > deadZone)
            return -driveController.getLeftX();
        else
            return 0;
    }

    @Override
    public double getRotatePower(){
        double rotatePower = -driveController.getRightX();
        if(Math.abs(rotatePower) > deadZone)
            return -driveController.getRightX();
        else
        return 0;
    }

    @Override
    public boolean getDriverHeadingResetButton(){
        return driveController.getL1Button();
    }

    @Override
    public boolean getLeftBumper(){
        return driveController.getL1Button();
    }

    @Override
    public boolean getRightBumper(){
        return driveController.getR1Button();
    }

    @Override
    public boolean getRelativeMode(){
        return driveController.getR1Button();
    }

    @Override
    public void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem
    ){
        EventLoop eventLoop = new EventLoop();
        eventLoop.bind(command);
        driveController.L1(eventLoop);
    }

    @Override
    public void setDeadZone(double deadZone){
        this.deadZone = deadZone;
    }
}
