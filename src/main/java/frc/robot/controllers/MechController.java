package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class MechController {
    public CommandPS5Controller controller;
    private double deadZone;
    public MechController(int port, double deadZone) {
        controller = new CommandPS5Controller(port);
        this.deadZone = deadZone;
    }

    public double getLeftX() {
        return Math.abs(controller.getLeftX()) > deadZone ? controller.getLeftX() : 0;
    }

    public double getLeftY() {
        return Math.abs(controller.getLeftY()) > deadZone ? controller.getLeftY() : 0;
    }

    public double getRightX() {
        return Math.abs(controller.getRightX()) > deadZone ? controller.getRightX() : 0;
    }

    public double getRightY() {
        return Math.abs(controller.getRightY()) > deadZone ? controller.getRightY() : 0;
    }

    public double getLeftTrigger() {
        return Math.abs(controller.getL2Axis()) > deadZone ? controller.getL2Axis() : 0;
    }

    public double getRightTrigger() {
        return Math.abs(controller.getR2Axis()) > deadZone ? controller.getR2Axis() : 0;
    }
}
