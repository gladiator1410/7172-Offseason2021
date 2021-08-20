package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotHardwareOBV2
{

    public final double INTAKE_POWER_INTAKE = 1;
    public final double INTAKE_POWER_OFF = 0;
    public final double INTAKE_POWER_OUTTAKE = -1.0;
    public final double CONVEYOR_POWER_INTAKE = 0.75;
    public final double CONVEYOR_POWER_FIRE = 0.90;
    public final double CONVEYOR_POWER_OUTTAKE = -0.75;
    public final double CONVEYOR_POWER_OFF = 0;
    public final double INDEXER_POSITION_OFF = 0.5;
    public final double INDEXER_POSITION_LOAD = 0.5;
    public final double INDEXER_POSITION_FIRE = 0.0;
    public final double SHOOTER_VELOCITY_HIGH = 1540;
    public final double SHOOTER_VELOCITY_MID = 1360;
    public final double SHOOTER_VELOCITY_PSHOT = 1380;
    public final double SHOOTER_VELOCITY_OFF = 0;
    public final double SHOOTER_VELOCITY_NORMAL = 1520;     // deprecated
    public final double SHOOTER_VELOCITY_LOW = 1360;        // deprecated
    public final double GRABBER_POSITION_CLOSE = 0.025;
    public final double GRABBER_POSITION_OPEN = 0.85;
    public final double WOBBLE_VELOCITY_STOW = -700;
    public final double WOBBLE_VELOCITY_FLIP = 700;
    public final double TILT_POSITION_INIT = 0.66;
    public final double TURRET_POSITION_MIN = 0.36;
    public final double TURRET_POSITION_MAX = 0.70;
    public final double TURRET_POSITION_STRAIGHT = 0.56;
    public final double DROP_DOWN_POS = 0.4;
    public final double DROP_UP_POS = 0.6;
    public final double CAM_RED_OUTTER_POS = 0.3103;
    public final double CAM_RED_INNER_POS = 0.57129;
    public final double CAM_BLUE_OUTTER_POS = 0.5747;
    public final double CAM_BLUE_INNER_POS = 0.4212;
    public double targetServoPos = 0.5;
    public double cameraPos = 0.5;
    public double goalWidth = 0;

    public enum ShootMode { IDLE, LOAD, TRIGGER, FIRE, RECOVER }
    public ShootMode smode = ShootMode.IDLE;
    public ElapsedTime smodeTimer = new ElapsedTime();
    public ElapsedTime robotTimer = new ElapsedTime();
    public int shots = 0;

    public static enum WGMode { IDLE, STOW, FLIP }
    public WGMode wgmode = WGMode.IDLE;
    public ElapsedTime wgmodeTimer = new ElapsedTime();

    //Create variables for hardware
    public DcMotor lf   = null;
    public DcMotor rf   = null;
    public DcMotor lb   = null;
    public DcMotor rb   = null;
    public DcMotor intake = null;
    public DcMotor conveyor = null;
    public Servo tilt = null;
    public Servo indexer = null;
    public DcMotorEx shooter1 = null;
    public DigitalChannel magnet = null;
    public NormalizedColorSensor colorv3 = null;
    public DcMotorEx wobble = null;
    public Servo grabber = null;
    public Servo targetServo = null;
    public Servo camera = null;
    public Servo colorServo = null;

    public DigitalChannel ledr0 = null;
    public DigitalChannel ledr1 = null;
    public DigitalChannel ledg0 = null;
    public DigitalChannel ledg1 = null;

    public BNO055IMU imu = null;

    //Create Hardware Map Object
    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    public double intakePower = 0.0;
    public double conveyorPower = 0.0;
    public double fireVelocity = SHOOTER_VELOCITY_HIGH;
    public double shooterAdjust = 0;
    public ElapsedTime intakeTimer = new ElapsedTime();

    public Servo turret = null;
    public double turretPosition = TURRET_POSITION_STRAIGHT;
    public double turretBase = TURRET_POSITION_STRAIGHT;
    public double turretVolt = 0;
    public double turretMult = 0;
    public double turretAdjust = 0.0;
    public int shootTarget = 0;
    public Servo drop = null;

    public OpenCvCamera webcam;
    public UGBasicHighGoalPipeline pipeline = new UGBasicHighGoalPipeline();

    public int targetColor = 3;
    double REDx1 = 143;
    double REDy1 = 0.525;
    double REDx2 = 240;
    double REDy2 = 0.63;
    public double goalPos = 0;
    boolean goalSeen = false;
    boolean goalTurn = false;

    double[][] redTargets = {
            {246, 0.672, 190, 0.624, 1500},
            {246, 0.672, 190, 0.624, 1400},
            {246, 0.552, 190, 0.464, 1380},
            {246, 0.584, 190, 0.504, 1380},
            {246, 0.608, 190, 0.552, 1380}
    };
    double[][] blueTargets = {
            {73, 0.496, 265, 0.696, 1500},
            {73, 0.496, 265, 0.696, 1400},
            {73, 0.584, 125, 0.616, 1380},
            {73, 0.616, 122, 0.648, 1380},
            {73, 0.640, 125, 0.700, 1380}
    };
    int target = 0;

    boolean LEDSet = true;

    //Initialize Hardware That
    //Comes from the Config
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotor.class, "lf");
        rf = hwMap.get(DcMotor.class, "rf");
        lb = hwMap.get(DcMotor.class, "lb");
        rb = hwMap.get(DcMotor.class, "rb");
        intake = hwMap.get(DcMotor.class, "intake");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        // tilt = hwMap.get(Servo.class, "tilt");
        indexer = hwMap.get(Servo.class, "indexer");
        shooter1 = (DcMotorEx)hwMap.get(DcMotor.class, "shooter1");
        colorv3 = hwMap.get(NormalizedColorSensor.class, "colorv3");
        wobble = (DcMotorEx)hwMap.get(DcMotor.class, "wobble");
        grabber = hwMap.get(Servo.class, "grabber");
        turret = hwMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.REVERSE);
        drop = hwMap.get(Servo.class, "drop");
        targetServo = hwMap.get(Servo.class, "targetServo");
        camera = hwMap.get(Servo.class, "camera");
        colorServo = hwMap.get(Servo.class, "target");

        // Set all motors to zero power
        //Set servos to starting position
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        intake.setPower(0);
        conveyor.setPower(0);
        // tilt.setPosition(TILT_POSITION_INIT);
        indexer.setPosition(INDEXER_POSITION_LOAD);
        shooter1.setPower(0);
        wobble.setPower(0);
        grabber.setPosition(GRABBER_POSITION_OPEN);
        targetServo.setPosition(targetServoPos);
        turretPosition = TURRET_POSITION_STRAIGHT;
        turret.setPosition(turretPosition);
        camera.setPosition(cameraPos);

        // Set all motors to run without encoders
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocityPIDFCoefficients(150, 0, 0, 13);
        wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dropUp();

        initIMU(hwMap);   // must come after drive motors initialized




        ledr0 = hwMap.get(DigitalChannel.class, "led4");
        ledr0.setMode(DigitalChannel.Mode.OUTPUT);
        ledr0.setState(LEDSet);
        ledg0 = hwMap.get(DigitalChannel.class, "led5");
        ledg0.setMode(DigitalChannel.Mode.OUTPUT);
        ledg0.setState(LEDSet);

        ledr1 = hwMap.get(DigitalChannel.class, "led6");
        ledr1.setMode(DigitalChannel.Mode.OUTPUT);
        ledr1.setState(LEDSet);
        ledg1 = hwMap.get(DigitalChannel.class, "led7");
        ledg1.setMode(DigitalChannel.Mode.OUTPUT);
        ledg1.setState(LEDSet);
    }

    public void startTargetingCamera(){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public void updateColorServo(){
        if(targetColor == 1){
            colorServo.setPosition(0);
        }
        else if(targetColor == 2){
            colorServo.setPosition(0.67);
        }
        else{
            colorServo.setPosition(1);
        }
    }

    public void setAutonCamera(double x){
        if(x == 1){
            cameraPos = CAM_RED_OUTTER_POS;
            camera.setPosition(cameraPos);
        }
        else if(x == 2){
            cameraPos = CAM_RED_INNER_POS;
            camera.setPosition(cameraPos);
        }
        else if(x == 3){
            cameraPos = CAM_BLUE_OUTTER_POS;
            camera.setPosition(cameraPos);
        }
        else if(x == 4){
            cameraPos = CAM_BLUE_INNER_POS;
            camera.setPosition(cameraPos);
        }
        else {
            cameraPos += x;
            camera.setPosition(cameraPos);
        }
    }

    public void initIMU(HardwareMap hwMap) {
        driveYXW(0,0,0);
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }

    public void intake() {
        intakePower = INTAKE_POWER_INTAKE;
        conveyorPower = CONVEYOR_POWER_INTAKE;
        intakeTimer.reset();
        if (smode == ShootMode.IDLE)
            setShootMode(ShootMode.LOAD);
    }

    public void stopIntake(){
        intakePower = 0;
        conveyorPower = 0;
    }

    public void outtake() {
        intakePower = INTAKE_POWER_OUTTAKE;
        conveyorPower = CONVEYOR_POWER_OUTTAKE;
    }

    public void setFireVelocity(double v) {
        if(v < 1900){
            fireVelocity = v;

        }
        else {
            fireVelocity = 1900;
        }
    }

    public void adjustShooter(double t) {
        shooterAdjust += t;
        if (t == 0) shooterAdjust = 0;
    }

    public void shooter(double v) {
        v += shooterAdjust;
        if (v < 400) v = 0;
        shooter1.setVelocity(v);
    }

    public void fire() {
        if (smode != ShootMode.FIRE && smode != ShootMode.RECOVER)
            setShootMode(ShootMode.TRIGGER);
    }

    public void quiet() {
        intakePower = INTAKE_POWER_OFF;
        conveyorPower = CONVEYOR_POWER_OFF;
        shooter(SHOOTER_VELOCITY_OFF);
        indexer.setPosition(INDEXER_POSITION_LOAD);
        setShootMode(ShootMode.IDLE);
    }

    public void setShootMode(ShootMode s) {
        if (smode != s) {
            if (telemetry != null)
                telemetry.log().add("%9.3f ShootMode %s", robotTimer.seconds(), s.toString());
            smode = s;
            smodeTimer.reset();
        }
    }

    public void updateAll() {

        updateTurret();

        double indexerPos = INDEXER_POSITION_OFF;
        if (smode == ShootMode.TRIGGER) {   // "fire" button requested
            conveyorPower = CONVEYOR_POWER_FIRE;
            indexerPos = INDEXER_POSITION_LOAD;
            shooter(fireVelocity);
            if (isFlyReady()) { setShootMode(ShootMode.FIRE); }
            else if (smodeTimer.seconds() > 1.0)
                setShootMode(ShootMode.LOAD);
        }
        if (smode == ShootMode.FIRE) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (isRingLoaded()) setShootMode(ShootMode.RECOVER);
            if (smodeTimer.seconds() > 0.5) {
                shots++; setShootMode(ShootMode.LOAD);
            }
        }
        if (smode == ShootMode.RECOVER) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (!isRingLoaded()) { shots++; setShootMode(ShootMode.LOAD); }
            else if (smodeTimer.seconds() > 0.2) { shots++; setShootMode(ShootMode.LOAD); }
        }
        if (smode == ShootMode.LOAD) {
            indexerPos = INDEXER_POSITION_LOAD;
            if (isRingLoaded()) shooter(fireVelocity);
        }

        updateWG();
        updateTargetServo();
        updateLED();
        updateColorServo();

        if (intakeTimer.seconds() < 1) conveyorPower = CONVEYOR_POWER_INTAKE;

        intake.setPower(motorPower(intakePower));
        conveyor.setPower(motorPower(conveyorPower));
        indexer.setPosition(indexerPos);

        intakePower = delatch(intakePower);
        conveyorPower = delatch(conveyorPower);
    }

    public double motorPower(double x) {
        // convert power values in 1.0..2.0 (latched) to be in 0..1.0.
        if (x > 1.0) return x-1;
        if (x < -1.0) return x+1;
        return x;
    }

    public double delatch(double x) {
        // keep x if it's latched, set to zero otherwise
        if (x < -1.0 || x > 1.0) return x;
        return 0;
    }

    public boolean isRingLoaded() {
        boolean s = ((DistanceSensor)colorv3).getDistance(DistanceUnit.CM) < 4;
        // if (s != led6.getState()) led6.setState(s);
        return s;
    }

    public boolean isFlyReady() {
        double vel = shooter1.getVelocity() - shooterAdjust;
        boolean s = (vel >= fireVelocity -20 && vel <= fireVelocity + 20);
        return s;
    }

    public boolean isShooterReady() {
        return isRingLoaded() && isFlyReady();
    }

    public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getHeading() {
        return getIMUHeading();
    }

    public void driveYXW(double ry, double rx, double rw) {
        // ry == forward, rx == strafe, rw == turn
        lf.setPower(ry + rw + rx);
        rf.setPower(ry - rw - rx);
        lb.setPower(ry + rw - rx);
        rb.setPower(ry - rw + rx);
    }


    public void wgStow() { setWGMode(WGMode.STOW); }

    public void wgFlip() { setWGMode(WGMode.FLIP); }

    public void wgOpen() {
        grabber.setPosition(GRABBER_POSITION_OPEN);
    }

    public void wgClose() {
        grabber.setPosition(GRABBER_POSITION_CLOSE);
    }

    public void setWGMode(WGMode wgm) {
        wgmode = wgm;
        wgmodeTimer.reset();
    }

    public void updateWG() {
        double wobbleVelocity = 0;
        if (wgmode == WGMode.STOW) {
            if (wgmodeTimer.seconds() < 1.5)
                wobbleVelocity = WOBBLE_VELOCITY_STOW;
        }
        if (wgmode == WGMode.FLIP) {
            if (wgmodeTimer.seconds() < 1.5)
                wobbleVelocity = WOBBLE_VELOCITY_FLIP;
        }
        wobble.setVelocity(wobbleVelocity);
    }

    // drive robot forward/strafe, maintain heading of th
    public void driveYXH(double ry, double rx, double th) {
        double herror = getHeading() - th;
        driveYXW(ry, rx, herror * 0.015);
    }

    public void setTurretPosition(double t) {
        turretPosition = Range.clip(t, TURRET_POSITION_MIN, TURRET_POSITION_MAX);
    }

    public void adjustTurret(double t) {
        turretAdjust += t;
        if (t == 0) turretAdjust = 0;
    }

    public void updateTurret() {
        if (targetColor == 1 || targetColor == 2) {
            turretPosition = getTargetPos(updateOpenCV());
            turretPosition += turretAdjust;
            turretPosition = Range.clip(turretPosition, TURRET_POSITION_MIN, TURRET_POSITION_MAX);
            turret.setPosition(turretPosition);
        }
        if (targetColor == 3) {
            turretPosition = TURRET_POSITION_STRAIGHT + turretAdjust;
            turretPosition = Range.clip(turretPosition, TURRET_POSITION_MIN, TURRET_POSITION_MAX);
            turret.setPosition(turretPosition);
        }
    }


    public void dropDown(){
        drop.setPosition(DROP_DOWN_POS);
    }

    public void dropUp(){
        drop.setPosition(DROP_UP_POS);
    }

    public int getTargetColor(){
        return targetColor;
    }

    public void setTargetColor(int x){
        targetColor = x;
        if(x == 3){
            turretPosition = TURRET_POSITION_STRAIGHT;
        }
    }

    public void switchColor(){
        if(targetColor == 1){
            targetColor = 2;
        }
        if(targetColor == 2){
            targetColor = 1;
        }
        if(targetColor == 3){
            targetColor = 1;
        }
    }

    public double updateOpenCV(){
        if (pipeline.isRedVisible() && targetColor == 1) {
            Rect redRect = pipeline.getRedRect();
            Point centerOfRedGoal = pipeline.getCenterofRect(redRect);
            goalWidth = redRect.width;
            goalSeen = false;
            goalTurn = false;
            goalPos = 0;

            if (redRect.width >= 60 && centerOfRedGoal.x - (0.5*redRect.width) >= 5 && centerOfRedGoal.x + (0.5*redRect.width) <= 315){
                goalSeen = true;
                goalTurn = false;
                goalPos = centerOfRedGoal.x;
            }
        }
        else if (pipeline.isBlueVisible() && targetColor == 2) {
            Rect blueRect = pipeline.getBlueRect();
            Point centerOfBlueGoal = pipeline.getCenterofRect(blueRect);
            goalWidth = blueRect.width;
            goalSeen = false;
            goalTurn = false;
            goalPos = 0;

            if (blueRect.width >= 60 && centerOfBlueGoal.x - (0.5*blueRect.width) >= 5 && centerOfBlueGoal.x + (0.5*blueRect.width) <= 315){
                goalSeen = true;
                goalTurn = false;
                goalPos = centerOfBlueGoal.x;
            }
        }
        else{
            goalPos = 0;
            goalSeen = false;
            goalTurn = true;
        }

        return goalPos;
    }

    public double getTargetPos(double x){

        if(goalSeen == false){
            return turretPosition;
        }

        else if(targetColor == 1 && x != 0 && goalSeen) {
            double slope = (redTargets[target][1] - redTargets[target][3])/(redTargets[target][0] - redTargets[target][2]);
            double yIntercept = redTargets[target][1] - slope*redTargets[target][0];
            setFireVelocity(redTargets[target][4]);
            double returnPos = slope*x + yIntercept - 0.016;

            return returnPos;
        }

        else if(targetColor == 2 && x != 0 && goalSeen) {
            double slope = (blueTargets[target][1] - blueTargets[target][3])/(blueTargets[target][0] - blueTargets[target][2]);
            double yIntercept = blueTargets[target][1] - slope*blueTargets[target][0];
            setFireVelocity(blueTargets[target][4]);
            double returnPos = slope*x + yIntercept;

            return returnPos;
        }

        return turretPosition;

    }


    public void updateTargetServo(){
        /*
        if(goalSeen && !goalTurn){
            targetServoPos = 0.5;
            goalTurn = false;
        }
        else {
            goalTurn = true;
            targetServoPos = 0.00417 * getHeading() + 0.49167;
        }
         */
        targetServoPos = 0.5;
        targetServo.setPosition(targetServoPos);
    }


    public double getTargetServoPos(){
        return targetServoPos;
    }

    public void setTarget(int x){
        target = x;
    }

    public void updateLED(){
        boolean red = true;      // default led4 (red) off
        boolean grn = true;      // default led5 (green) off

        if(targetColor == 3){
            goalSeen = false;
        }

        if (goalSeen) grn = false;
        else red = false;

        if (ledr0.getState() != red) ledr0.setState(red);
        if (ledr1.getState() != red) ledr1.setState(red);
        if (ledg0.getState() != grn) ledg0.setState(grn);
        if (ledg1.getState() != grn) ledg1.setState(grn);
    }

    public void autoFire(){
        fire();
        while(smode != ShootMode.LOAD){
            updateAll();
        }
        telemetry.update();
    }
}


