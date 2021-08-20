package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RobotHardwareAS extends RobotHardwareOB
{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AcvtjYf/////AAABmehCu3IEgElchwvMigjFqD1s9omMDs5F01lEo3FVqEIg/l5XQlHGj9MgXekDJiOt9m4WamftoEqEUUHlx9pbqW01bmole7jyWAU50dipOfpJ75c4k04Bnscb6RJkbcacd9JpgNTNngaCJiYJ3E6ZyJ3ay4pvwnBmcLPopk+UbI/igXNbCX0TVWED91OwFgy/aRIW2o3srpk9ACTqOG7CH8AzABCbQljzv5ML+B6lwCK8vGTAO1pABAdDC/KCArkjbWKMvbI3lDnNHA4mWuNi1zsO6XZZss5t+3FtnqY2iW078V5YQHOEOnldSTQjfW65/L6NYgm4yHT8GZSRiS7U4eoWBgumFDM9TFXOYZ74MFJU";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;




    public void driveYXW(double ry, double rx, double rw) {
        // ry == forward, rx == strafe, rw == turn
        lf.setPower(ry + rw + rx);
        rf.setPower(ry - rw - rx);
        lb.setPower(ry + rw - rx);
        rb.setPower(ry - rw + rx);
    }

    public void driveYXH(double ry, double rx, double th, double ch) {
        double h = ch - th;
        driveYXW(ry, rx, h * 0.02);
    }

    public void updateAll(int velocity) {
        double indexerPos = INDEXER_POSITION_OFF;
        if (smode == ShootMode.TRIGGER) {   // "fire" button requested
            conveyorPower = CONVEYOR_POWER_FIRE;
            indexerPos = INDEXER_POSITION_LOAD;
            shooter(velocity);
            if (isShooterReady()) setShootMode(ShootMode.FIRE);
            if (smodeTimer.seconds() > 1.0)
                setShootMode(ShootMode.LOAD);
        }
        if (smode == ShootMode.FIRE) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (smodeTimer.seconds() > 0.2) {
                setShootMode(ShootMode.LOAD);
            }
        }
        if (smode == ShootMode.LOAD) {
            indexerPos = INDEXER_POSITION_LOAD;
            if (isRingLoaded()) shooter(velocity);
        }

        updateWG();

        if (intakeTimer.seconds() < 1) conveyorPower = CONVEYOR_POWER_INTAKE;

        intake.setPower(motorPower(intakePower));
        conveyor.setPower(motorPower(conveyorPower));
        indexer.setPosition(indexerPos);

        intakePower = delatch(intakePower);
        conveyorPower = delatch(conveyorPower);
    }

    public boolean isShooterReady(int velocity) {
        double vel = shooter1.getVelocity();
        return isRingLoaded()
                && vel >= velocity - 20
                && vel <= velocity + 20;
    }



    public void driveYDH(double ry, double dv, double th, double heading) {
        double herror = heading - th;
        double derror = dv - getLRangeV();
        if (herror < -10 || herror > 10) derror = 0;
        driveYXW(ry, derror * 25, herror * 0.02);
    }

    public void initVision(){
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public String UpdateVision() {
        String result = "";

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    result = recognition.getLabel();
                }
            }
        }

        return result;
    }





}

