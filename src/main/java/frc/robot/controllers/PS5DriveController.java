package frc.robot.controllers;

import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PS5DriveController extends BaseDriveController{

    private final PS5Controller driveController = new PS5Controller(0);
    
    @Override
    public double getForwardPower() {
        return -driveController.getLeftY();
    }

    @Override
    public double getLeftPower(){
        return -driveController.getLeftX();
    }

    @Override
    public double getRotatePower(){
        return -driveController.getRightX();
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
}
