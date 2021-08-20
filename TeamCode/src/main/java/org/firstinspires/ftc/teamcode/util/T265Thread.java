package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class T265Thread extends Thread {
    private String CameraPos;
    private String left = "left";
    private String right = "right";
    private String back = "back";
    private String front = "front";

    private static T265Camera slamra = null;
    RobotHardware robot   = new RobotHardware();
    public HardwareMap hardwareMap;

    public double Y = 0;
    public double X = 0;
    public double Heading = 0;

    public T265Thread(String aPos)
    {
        this.setName("T265Thread");
        CameraPos = aPos;
    }

    public T265Thread()
    {
        this.setName("T265Thread");
        CameraPos = "left";
    }

    @Override
    public void run()
    {
        robot.init(hardwareMap);

        initCameraPos();

        slamra.start();

        while (!isInterrupted())
        {
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

            if (up == null) return;



            if(CameraPos.equals(left)){
                // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                Y = (-1)*(up.pose.getTranslation().getY() / 0.0254);
                X = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                Heading = (up.pose.getHeading()) * (-57.295);
            }

            if(CameraPos.equals(right)){
                // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                Y = (up.pose.getTranslation().getY() / 0.0254);
                X = (up.pose.getTranslation().getX() / 0.0254);
                Heading = (up.pose.getHeading()) * (-57.295);
            }

            if(CameraPos.equals(back)){
                // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                Y = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                X = (up.pose.getTranslation().getY() / 0.0254);
            }

            if(CameraPos.equals(front)){
                // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                Y = (up.pose.getTranslation().getX() / 0.0254);
                X = (-1)*(up.pose.getTranslation().getY() / 0.0254);
            }


            Translation2d translation = new Translation2d(X, Y);
            Rotation2d rotation = up.pose.getRotation();
        }

        slamra.stop();
    }

    public void initCameraPos(){
        for(int i = 0; i < 2; i++) {
            if (slamra == null) {
                slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            }
        }

        if(CameraPos.equals(left)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-31 * 0.0254, -12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(right)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(31 * 0.0254, 12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(back)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-12 * 0.0254, 31 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(front)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(12 * 0.0254, -31 * 0.0254, Rotation2d.fromDegrees(0)));
        }
    }
}
