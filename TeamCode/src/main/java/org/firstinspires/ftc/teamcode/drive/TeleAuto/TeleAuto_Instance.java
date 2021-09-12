package org.firstinspires.ftc.teamcode.drive.TeleAuto;

public class TeleAuto_Instance {

    private double time;

    private boolean x;
    private boolean y;
    private boolean a;
    private boolean b;
    private boolean xShift;
    private boolean yShift;
    private boolean aShift;
    private boolean bShift;

    private boolean dpad_left;
    private boolean dpad_right;
    private boolean dpad_up;
    private boolean dpad_down;
    private boolean dpad_leftShift;
    private boolean dpad_rightShift;
    private boolean dpad_upShift;
    private boolean dpad_downShift;

    private boolean leftBumper;
    private boolean rightBumper;
    private boolean back;
    private boolean left_bumperShift;
    private boolean right_bumperShift;
    private boolean backShift;

    private double leftStickX;
    private double rightStickX;
    private double leftStickY;
    private double rightStickY;

    private double rightTrigger;

    public TeleAuto_Instance(double time, boolean x, boolean y, boolean a, boolean b, boolean xShift, boolean yShift, boolean aShift, boolean bShift,
                             boolean dpad_left, boolean dpad_right, boolean dpad_up, boolean dpad_down, boolean dpad_leftShift, boolean dpad_rightShift,
                             boolean dpad_upShift, boolean dpad_downShift, boolean leftBumper, boolean rightBumper, boolean back, boolean left_bumperShift,
                             boolean right_bumperShift, boolean backShift, double leftStickX, double rightStickX, double leftStickY, double rightStickY, double rightTrigger){

        this.x = x;
        this.y = y;
        this.a = a;
        this.b = b;
        this.aShift = aShift;
        this.bShift = bShift;
        this.xShift = xShift;
        this.yShift = yShift;

        this.dpad_down = dpad_down;
        this.dpad_left = dpad_left;
        this.dpad_right = dpad_right;
        this.dpad_up = dpad_up;
        this.dpad_downShift = dpad_downShift;
        this.dpad_leftShift = dpad_leftShift;
        this.dpad_rightShift = dpad_rightShift;
        this.dpad_upShift = dpad_upShift;

        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.back = back;
        this.left_bumperShift = left_bumperShift;
        this.right_bumperShift = right_bumperShift;
        this.backShift = backShift;

        this.leftStickX = leftStickX;
        this.rightStickX = rightStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;

        this.rightTrigger = rightTrigger;

    }


}
