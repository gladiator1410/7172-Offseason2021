package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardwareOB
{

    public final double INTAKE_POWER_INTAKE = 1;
    public final double INTAKE_POWER_OFF = 0;
    public final double INTAKE_POWER_OUTTAKE = -1.0;
    public final double CONVEYOR_POWER_INTAKE = 0.80;
    public final double CONVEYOR_POWER_FIRE = 0.90;
    public final double CONVEYOR_POWER_OUTTAKE = -0.75;
    public final double CONVEYOR_POWER_OFF = 0;
    public final double INDEXER_POSITION_OFF = 0.5;
    public final double INDEXER_POSITION_LOAD = 0.5;
    public final double INDEXER_POSITION_FIRE = 0.0;
    public final double SHOOTER_VELOCITY_HIGH = 1520;
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
    public final double CHASSIS_RPIXY_TOWER = 1.946;    // rpixy value when aimed at red tower goal
    public final double CHASSIS_BPIXY_TOWER = 1.764;    // bpixy value when aimed at blue tower goal
    public final double CHASSIS_PSHOT_NEAR = 0.345;     // offset for pshot next to tower goal
    public final double CHASSIS_PSHOT_MID = 0.533;      // offset for pshot mid tower goal
    public final double CHASSIS_PSHOT_FAR = 0.720;      // offset for pshot away from tower goal
    public final double DROP_DOWN_POS = 0.4;
    public final double DROP_UP_POS = 0.6;
    public final double WHEELS_OFF = 0;
    public final double WHEELS_IN = 1;
    public final double WHEELS_OUT = -1;

    public final double TARGETS[][] = {
            // red targets (flyvelocity, pixy_a, pixy_b, turret_a, turret_b, pixy_c)
            { SHOOTER_VELOCITY_HIGH, 0.99, 2.528, 0.36, 0.70, CHASSIS_RPIXY_TOWER },  // 0: red high goal
            { SHOOTER_VELOCITY_MID, 0.99, 2.528, 0.36, 0.70, CHASSIS_RPIXY_TOWER },  // 1: red mid goal
            { SHOOTER_VELOCITY_PSHOT, 2.213, 2.919, 0.46, 0.60, CHASSIS_RPIXY_TOWER + CHASSIS_PSHOT_FAR },  // 2: red left powershot
            { SHOOTER_VELOCITY_PSHOT + 20, 2.024, 2.744, 0.46, 0.60, CHASSIS_RPIXY_TOWER + CHASSIS_PSHOT_MID },  // 3: red center powershot
            { SHOOTER_VELOCITY_PSHOT, 1.856, 2.534, 0.46, 0.60, CHASSIS_RPIXY_TOWER + CHASSIS_PSHOT_NEAR },  // 4: red right powershot
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_STRAIGHT, TURRET_POSITION_STRAIGHT, CHASSIS_RPIXY_TOWER },
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_MIN+0.0, TURRET_POSITION_MIN+0.0, 1.65 },
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_MAX-0.0, TURRET_POSITION_MAX-0.0, 1.65 },

            // blue targets (flyvelocity, pixy_a, pixy_b, turret_a, turret_b, pixy_c)
            { SHOOTER_VELOCITY_HIGH, 0.755, 2.34, 0.36, 0.70, CHASSIS_BPIXY_TOWER },  // 8: blue high goal
            { SHOOTER_VELOCITY_MID, 0.755, 2.34, 0.36, 0.70, CHASSIS_BPIXY_TOWER },  // 9: blue mid goal
            { SHOOTER_VELOCITY_PSHOT, 0.906, 1.533, 0.46, 0.60, CHASSIS_BPIXY_TOWER - CHASSIS_PSHOT_NEAR },  // 10: blue left powershot
            { SHOOTER_VELOCITY_PSHOT, 0.704, 1.372, 0.46, 0.60, CHASSIS_BPIXY_TOWER - CHASSIS_PSHOT_MID },  // 11: blue center powershot
            { SHOOTER_VELOCITY_PSHOT, 0.501, 1.210, 0.46, 0.60, CHASSIS_BPIXY_TOWER - CHASSIS_PSHOT_FAR },  // 12: blue right powershot
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_STRAIGHT, TURRET_POSITION_STRAIGHT, CHASSIS_BPIXY_TOWER },
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_MIN+0.0, TURRET_POSITION_MIN+0.0, 1.65 },
            { SHOOTER_VELOCITY_HIGH, 0, 3.3, TURRET_POSITION_MAX-0.0, TURRET_POSITION_MAX-0.0, 1.65 },
    };

    public final double LARM_POSITION_UP = 0.5;
    public final double LARM_POSITION_DOWN = 0.75;
    public final double RARM_POSITION_UP = 0.5;
    public final double RARM_POSITION_DOWN = 0.15;

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

    public Servo wheel1 = null;
    public Servo wheel2 = null;

    public Servo larm = null;
    public Servo rarm = null;

    public DigitalChannel ledr0 = null;
    public DigitalChannel ledr1 = null;
    public DigitalChannel ledg0 = null;
    public DigitalChannel ledg1 = null;

    public BNO055IMU imu = null;
    public BNO055IMU imu1 = null;

    public AnalogInput lrange = null;
    public double lrangeV = 0;

    public Pixy2 rpixy = null;
    public Pixy2 bpixy = null;
    public Pixy2 activePixy = null;
    public PIDF pixyPID = null;
    public double pixyTargetV = 0;
    public double drivelastrx = 0;

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
        larm = hwMap.get(Servo.class, "larm");
        rarm = hwMap.get(Servo.class, "rarm");
        shooter1 = (DcMotorEx)hwMap.get(DcMotor.class, "shooter1");
        colorv3 = hwMap.get(NormalizedColorSensor.class, "colorv3");
        wobble = (DcMotorEx)hwMap.get(DcMotor.class, "wobble");
        grabber = hwMap.get(Servo.class, "grabber");
        turret = hwMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.REVERSE);
        drop = hwMap.get(Servo.class, "drop");
        targetServo = hwMap.get(Servo.class, "target");

        wheel1 = hwMap.get(Servo.class, "wheel1");
        wheel2 = hwMap.get(Servo.class, "wheel2");

        lrange = hwMap.get(AnalogInput.class, "lrange");
        rpixy = new Pixy2();
        rpixy.initialize(hwMap, "rpixytrig", "rpixyvolt");
        bpixy = new Pixy2();
        bpixy.initialize(hwMap, "bpixytrig", "bpixyvolt");
        pixyPID = new PIDF();
        pixyPID.setPIDF(0.4, 0.06, 0.6, 0);
        setTarget(0, 0);

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
        wheel1.setPosition(0.5);
        wheel2.setPosition(0.5);
        grabber.setPosition(GRABBER_POSITION_OPEN);

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
        ledr0.setState(true);
        ledg0 = hwMap.get(DigitalChannel.class, "led5");
        ledg0.setMode(DigitalChannel.Mode.OUTPUT);
        ledg0.setState(true);

        ledr1 = hwMap.get(DigitalChannel.class, "led6");
        ledr1.setMode(DigitalChannel.Mode.OUTPUT);
        ledr1.setState(true);
        ledg1 = hwMap.get(DigitalChannel.class, "led7");
        ledg1.setMode(DigitalChannel.Mode.OUTPUT);
        ledg1.setState(true);
    }

    public void initIMU(HardwareMap hwMap) {
        driveYXW(0,0,0);
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        imu1 = hwMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(params);
    }

    public void intake() {
        intakePower = INTAKE_POWER_INTAKE;
        conveyorPower = CONVEYOR_POWER_INTAKE;
        intakeTimer.reset();
        if (smode == ShootMode.IDLE)
            setShootMode(ShootMode.LOAD);
    }

    public void outtake() {
        intakePower = INTAKE_POWER_OUTTAKE;
        conveyorPower = CONVEYOR_POWER_OUTTAKE;
    }

    public void setFireVelocity(double v) {
        fireVelocity = v;
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

    public double getIMU1Heading() {
        Orientation angles = imu1.getAngularOrientation(
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

    public double getLRangeV() {
        return lrange.getVoltage();
    }

    // drive robot forward/strafe, maintain heading of th
    public void driveYXH(double ry, double rx, double th) {
        double herror = getHeading() - th;
        driveYXW(ry, rx, herror * 0.015);
    }

    // drive robot forward, maintain heading th and distance from wall dv
    public void driveYDH(double ry, double dv, double th) {
        double herror = getHeading() - th;
        double derror = dv - getLRangeV();
        if (herror < -10 || herror > 10) derror = 0;
        driveYXW(ry, derror * 25, herror * 0.02);
    }

    public void setArms(boolean down) {
        larm.setPosition(down ? LARM_POSITION_DOWN : LARM_POSITION_UP);
        rarm.setPosition(down ? RARM_POSITION_DOWN : RARM_POSITION_UP);
    }


    public void setTarget(int pixy, int t) {
        double targetrow[] = TARGETS[t];
        shootTarget = t;
        targetServo.setPosition(0.07+t*0.14);

        if (pixy > 0) {
            activePixy = bpixy;
            shootTarget = t + 8;
            targetrow = TARGETS[shootTarget];
        }
        else if (pixy < 0) activePixy = null;
        else activePixy = rpixy;
        setFireVelocity(targetrow[0]);
        turretVolt = targetrow[1];
        turretBase = targetrow[3];
        turretMult = (targetrow[4]-targetrow[3])/(targetrow[2]-targetrow[1]);
        pixyTargetV = targetrow[5];
    }

    public void setTurretPosition(double t) {
        turretPosition = Range.clip(t, TURRET_POSITION_MIN, TURRET_POSITION_MAX);
    }

    public void adjustTurret(double t) {
        turretAdjust += t;
        if (t == 0) turretAdjust = 0;
    }

    public void updateTurret() {
        if (activePixy != null) {
            double v = activePixy.getValue();
            if (v >= 0)
                turretPosition = turretBase + (v - turretVolt) * turretMult + turretAdjust;
        }
        turretPosition = Range.clip(turretPosition, TURRET_POSITION_MIN, TURRET_POSITION_MAX);
        turret.setPosition(turretPosition + 0.01125);
    }

    // drive robot forward/strafe, maintain heading of pixy center
    public void driveYXP(double ry, double rx) {
        double turn = 0;
        if (drivelastrx > 0 && rx < 0.01) pixyPID.zeroI();
        drivelastrx = rx;
        double v = -100;
        if (activePixy != null) v = activePixy.getValue();
        if (v >= 0) turn = pixyPID.calc(v, pixyTargetV);
        driveYXW(ry, rx, turn);
    }


    public void allPowerShot(LinearOpMode op, int pixysel) {
        ElapsedTime powerTimer = new ElapsedTime();
        driveYXW(0,0,0);
        setTarget(pixysel, 2);
        powerTimer.reset();
        while (op.opModeIsActive() && powerTimer.seconds() < 0.5) updateAll();
        fire();
        while (op.opModeIsActive() && smode != ShootMode.LOAD) updateAll();
        setTarget(pixysel, 3);
        powerTimer.reset();
        while (op.opModeIsActive() && powerTimer.seconds() < 0.5) updateAll();
        fire();
        while (op.opModeIsActive() && smode != ShootMode.LOAD) updateAll();
        setTarget(pixysel, 4);
        powerTimer.reset();
        while (op.opModeIsActive() && powerTimer.seconds() < 0.5) updateAll();
        fire();
    }

    public void dropDown(){
        drop.setPosition(DROP_DOWN_POS);
    }

    public void dropUp(){
        drop.setPosition(DROP_UP_POS);
    }

}

