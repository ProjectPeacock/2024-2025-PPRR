package org.firstinspires.ftc.teamcode.hardware;

/****
 * DEPRECATED CLASS. Do Not Use!
 */




import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RRHWProfile {

    /* Public OpMode members. */
    public MotorEx motorLF = null;
    //
    public MotorEx motorLR = null;
    //
    public MotorEx motorRF = null;
    //1
    public MotorEx motorRR = null;
    //2

    public AnalogInput sensorDistanceFL = null;
    public AnalogInput sensorDistanceFR = null;

    public DcMotorEx auto_MotorLiftFront = null;

    public DcMotorEx motorIntake = null;
    //3
    public DcMotorEx odoPerp = null;
    public MotorEx motorLiftFront = null;
    public MotorEx motorLiftBack = null;
    public MotorGroup lift=null;

    public Servo servoIntakeLeft = null;
    public Servo servoIntakeRight = null;
    //port0CH
    public Servo servoArm= null;
    public Servo servoPlunger = null;
    public Servo servoPlungerAngle = null;
    public CRServo servoAgitator = null;
    //port2CH
    public Servo servoDrone = null;
    //port1CH

    public RevIMU imu = null;
    //

    public DigitalChannel beamBucketBottom = null;
    //
    public DigitalChannel beamBucketTop = null;
    //

    public MecanumDrive mecanum = null;
    // public MotorEx autoLight = null;

    public DigitalChannel ledLeftFrontRed = null;
    //
    public DigitalChannel ledLeftFrontGreen = null;
    //
    public DigitalChannel ledRightFrontRed = null;
    //
    public DigitalChannel ledRightFrontGreen = null;
    //
    public DigitalChannel ledLeftRearRed = null;
    //
    public DigitalChannel ledLeftRearGreen = null;
    //
    public DigitalChannel ledRightRearRed = null;
    //
    public DigitalChannel ledRightRearGreen = null;
    //
    public Boolean opModeTeleop = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public RRHWProfile(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean teleOp) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.opModeTeleop = teleOp;

        /*
         * Initialize Motors
         */

        if(opModeTeleop){
            //drive motor init
            motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
            motorLF.setRunMode(Motor.RunMode.RawPower);
            motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorLF.resetEncoder();

            motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
            motorLR.setRunMode(Motor.RunMode.RawPower);
            motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorLR.resetEncoder();

            motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
            motorRF.setRunMode(Motor.RunMode.RawPower);
            motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorRF.resetEncoder();

            motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
            motorRR.setRunMode(Motor.RunMode.RawPower);
            motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorRR.resetEncoder();

            motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", 385,435);
            motorLiftBack = new MotorEx(ahwMap, "motorLiftBack", 385,435);
            motorLiftBack.resetEncoder();

            odoPerp = hwMap.get(DcMotorEx.class, "odoPerp");
            odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lift = new MotorGroup(motorLiftFront, motorLiftBack);
            lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            lift.set(0);
            lift.resetEncoder();

            //init imu
            imu = new RevIMU(ahwMap);
            imu.init();

            //drivebase init
            mecanum = new com.arcrobotics.ftclib.drivebase.MecanumDrive(motorLF, motorRF, motorLR, motorRR);
        } else {

            auto_MotorLiftFront = hwMap.get(DcMotorEx.class, "motorLiftFront");
            auto_MotorLiftFront.setDirection(DcMotor.Direction.FORWARD);
            auto_MotorLiftFront.setTargetPosition(0);
            auto_MotorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            auto_MotorLiftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            auto_MotorLiftFront.setPower(0);
        }

        motorIntake = hwMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorIntake.setPower(0);

        servoIntakeLeft = hwMap.servo.get("servoIntakeLeft");
        servoIntakeRight = hwMap.servo.get("servoIntakeRight");
        servoArm = hwMap.servo.get("servoArm");
        servoPlungerAngle = hwMap.servo.get("servoPlungerAngle");
        servoPlunger = hwMap.servo.get("servoPlunger");
        servoDrone = hwMap.get(ServoImplEx.class, "servoDrone");

        servoAgitator = hwMap.crservo.get("servoAgitator");

        beamBucketBottom = hwMap.get(DigitalChannel.class, "beamBucketBottom");
        beamBucketTop = hwMap.get(DigitalChannel.class, "beamBucketTop");

        ledLeftFrontGreen = hwMap.get(DigitalChannel.class, "ledLFGreen");
        ledLeftFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftFrontRed = hwMap.get(DigitalChannel.class, "ledLFRed");
        ledLeftFrontRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftRearGreen = hwMap.get(DigitalChannel.class, "ledLRGreen");
        ledLeftRearGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftRearRed = hwMap.get(DigitalChannel.class, "ledLRRed");
        ledLeftRearRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightFrontGreen = hwMap.get(DigitalChannel.class, "ledRFGreen");
        ledRightFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightFrontRed = hwMap.get(DigitalChannel.class, "ledRFRed");
        ledRightFrontRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightRearGreen = hwMap.get(DigitalChannel.class, "ledRRGreen");
        ledRightRearGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightRearRed = hwMap.get(DigitalChannel.class, "ledRRRed");
        ledRightRearRed.setMode(DigitalChannel.Mode.OUTPUT);

        sensorDistanceFL = hwMap.get(AnalogInput.class, "DistanceFL");
        sensorDistanceFR = hwMap.get(AnalogInput.class, "DistanceFR");

    }   // end of init() method

}       // end of the HardwareProfile class