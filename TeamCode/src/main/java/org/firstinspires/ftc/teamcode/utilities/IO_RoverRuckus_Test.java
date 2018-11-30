package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;

/**
 * Created by David Austin on 10/27/2016.
 */

public class IO_RoverRuckus_Test {
    public DcMotor rightBackDrive, leftBackDrive, chinMotor, dom1Motor, dom2Motor, domExtendMotor, domSweepMotor;
    //public GyroSensor gyro;
    public CRServo hook;
    public Servo markerBox;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;
    public DistanceSensor frontDistance;
    public DistanceSensor backDistance;

    public DigitalChannel touchChin;
    public DigitalChannel touchDOM;
    public DigitalChannel touchDOMExtend;

    public AnalogInput domPot;

    public WebcamName webcamName;

    public BNO055IMU imu;
    public BNO055IMU imu1;
    //double gyroOffset = 0;
    double imuOffset = 0;
    double imu1Offset = 0;
    public double heading = 0;
    public double heading1 = 0;
    double rightBackOffset = 0, leftBackOffset = 0, chinOffset = 0, dom1Offset = 0, dom2Offset = 0, domExtendOffset = 0;
    double lastRightBackEncoder = 0, lastLeftBackEncoder = 0, lastChinEncoder = 0; //lastDOM1Encoder = 0, lastDOM2Encoder = 0, lastDOMExtendEncoder = 0;
    double x = 0, y = 0;
    double COUNTSPERINCH = 140/1.28;//84/1.28;
    public double DEGREESPERVOLT = 270/3.3; //potentiometer
    public static int RED = 1, BLUE = 2;
    public static int UNKNOWN = 0, LEFT = 1, CENTER = 2, RIGHT = 3;
    public static int allianceColor = UNKNOWN;
    //public int vuMark = UNKNOWN;
    public static int cameraMonitorViewId;
    //public int jewelColor = UNKNOWN;

    //public double leftProximityAverage = 0;
    //public double rightProximityAverage = 0;
    //public double proximityCorrection = 0;
    //public boolean proximityArmButtonPushed = false;
    public boolean isGoldFound = false;
    public boolean isGoldAligned = false;
    public boolean isGoldCentered = false;
    public boolean lastIsGoldFound = false;
    public boolean lastIsGoldAligned = false;
    public boolean lastIsGoldCentered = false;

    public boolean twoCyclesIsGoldFound = false;
    public boolean twoCyclesIsGoldAligned = false;
    public boolean twoCyclesIsGoldCentered = false;

    public boolean completedSearch = false;
    public boolean goldMineralFound = false;

    public boolean isGoldTheCenterMineral = false;
    public boolean isGoldTheLeftMineral = false;
    public boolean isGoldTheRightMineral = false;

    public double goldXPosition = 0;
    //public double headingatlanding = 0;
    public double headingOfGold = 0;

    public static double hookClockwise = .79;
    public static double hookCounterClockwise = -.79;
    public static double hookStop = 0;

    //public static double leftHandOut = 0;
    //public static double rightHandOut = 1;
    //public static double leftHandMid = .3;
    //public static double rightHandMid = .7;
    ///public static double leftHandIn = 1;
    //public static double rightHandIn = 0;
    //public static double jewelArmUp = 1;
    //public static double jewelArmDown = 0;

    //public static double relicHandOpen = 0;
    //public static double relicHandClosed = 1;

    public static double markerBoxUp = 0;
    public static double markerBoxFlat = .5;
    public static double markerBoxDown = 1;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    public Orientation angles1;
    public Acceleration gravity1;
    public double[] eulerAngles = new double[3];
    public double[] eulerAngles1 = new double[3];

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Telemetry telemetry;
    HardwareMap map;

    public IO_RoverRuckus_Test(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = map;
        rightBackDrive = map.dcMotor.get("backright");
        leftBackDrive = map.dcMotor.get("backleft");
        chinMotor = map.dcMotor.get("chin");
        dom1Motor = map.dcMotor.get("dom1");
        dom2Motor = map.dcMotor.get("dom2");
        domExtendMotor = map.dcMotor.get("domExtend");
        domSweepMotor = map.dcMotor.get("domSweep");

        hook = map.crservo.get("hook");
        markerBox = map.servo.get("markerbox");
        /*rightHand = map.servo.get("right_hand");
        jewelArm = map.servo.get("jewel_arm");
        proximityArm = map.servo.get("proximity_arm");
        relicHand = map.servo.get("relic_hand");*/

        touchChin = map.digitalChannel.get("touchchin");
        touchDOM = map.digitalChannel.get("touchdom");
        touchDOMExtend = map.digitalChannel.get("touchdomextend");

        domPot = map.analogInput.get("dompot");

        webcamName = map.get(WebcamName.class, "Webcam 2");
        /*touchProximity = map.digitalChannel.get("touchproximity");
        touchLowerRelicArm = map.digitalChannel.get("touchlowerrelicarm");
        touchUpperRelicArm = map.digitalChannel.get("touchupperrelicarm");*/

        //gyro = map.gyroSensor.get("gyro");
        imu = map.get(BNO055IMU.class, "imu");
        imu1 = map.get(BNO055IMU.class, "imu1");

        /*//ods = map.opticalDistanceSensor.get("ods");
        colorSensor = map.colorSensor.get("colorsensor");*/


        /*// get a reference to the color sensor.
        leftColor = map.get(ColorSensor.class, "leftcolor");*/

        // get a reference to the distance sensor that shares the same name.
        leftDistance = map.get(DistanceSensor.class, "leftdistance");
        rightDistance = map.get(DistanceSensor.class, "rightdistance");
        frontDistance = map.get(DistanceSensor.class, "frontdistance");
        backDistance = map.get(DistanceSensor.class, "backdistance");

        /*// get a reference to the color sensor.
        rightColor = map.get(ColorSensor.class, "rightcolor");

        // get a reference to the distance sensor that shares the same name.
        rightDistance = map.get(DistanceSensor.class, "rightcolor");

        // get a reference to the color sensor.
        revColorSensor = map.get(ColorSensor.class, "jewelcolor");

        // get a reference to the distance sensor that shares the same name.
        jewelDistance = map.get(DistanceSensor.class, "jewelcolor");*/

        touchChin.setMode(DigitalChannel.Mode.INPUT);
        touchDOM.setMode(DigitalChannel.Mode.INPUT);
        touchDOMExtend.setMode(DigitalChannel.Mode.INPUT);
        /*touchProximity.setMode(DigitalChannel.Mode.INPUT);
        touchLowerRelicArm.setMode(DigitalChannel.Mode.INPUT);
        touchUpperRelicArm.setMode(DigitalChannel.Mode.INPUT);*/

        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        chinMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dom1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        dom2Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        domExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        domSweepMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.useExternalCrystal   = true;
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    /*public void retractHands() {
        leftHand.setPosition(IO_RoverRuckus_Test.leftHandOut);
        rightHand.setPosition(IO_RoverRuckus_Test.rightHandOut);
    }

    public void retractHandsMid() {
        leftHand.setPosition(IO_RoverRuckus_Test.leftHandMid);
        rightHand.setPosition(IO_RoverRuckus_Test.rightHandMid);
    }

    public void closeHands() {
        leftHand.setPosition(IO_RoverRuckus_Test.leftHandIn);
        rightHand.setPosition(IO_RoverRuckus_Test.rightHandIn);
    }*/

    public void hookStop() {
        hook.setPower(IO_RoverRuckus_Test.hookStop);
    }

    public void hookClockwise() {
        hook.setPower(IO_RoverRuckus_Test.hookClockwise);
    }

    public void hookCounterClockwise() { hook.setPower(IO_RoverRuckus_Test.hookCounterClockwise); }

    public void markerBoxUp() { markerBox.setPosition(IO_RoverRuckus_Test.markerBoxUp); }

    public void markerBoxDown() { markerBox.setPosition(IO_RoverRuckus_Test.markerBoxDown); }

    public void markerBoxFlat() { markerBox.setPosition(IO_RoverRuckus_Test.markerBoxFlat); }


    /*public void jewelArmDown() { jewelArm.setPosition(IO_RoverRuckus_Test.jewelArmDown); }

    public void proximityArmUp() {
        proximityArm.setPosition(IO_RoverRuckus_Test.proximityArmUp);
    }

    public void proximityArmMid() {
        proximityArm.setPosition(IO_RoverRuckus_Test.proximityArmMid);
    }*/

    /*public void proximityArmDown() {
        proximityArm.setPosition(IO_RoverRuckus_Test.proximityArmDown);
    }*/

/*    public void setGyroOffset() {
        gyroOffset = gyro.getHeading();
    }*/
    public void setIMUOffset() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imuOffset = angles.firstAngle;
    }

    public void setIMU1Offset() {
        angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu1Offset = angles1.firstAngle;
    }

    public void resetDriveEncoders() {
        rightBackOffset = rightBackDrive.getCurrentPosition();
        leftBackOffset = leftBackDrive.getCurrentPosition();
        chinOffset = chinMotor.getCurrentPosition();
        dom1Offset = dom1Motor.getCurrentPosition();
        dom2Offset = dom2Motor.getCurrentPosition();
        domExtendOffset = domExtendMotor.getCurrentPosition();

        lastRightBackEncoder = 0;
        lastLeftBackEncoder = 0;
        lastChinEncoder = 0;
        //lastDOM1Encoder = 0;
        //lastDOM2Encoder = 0;
        //lastDOMExtendEncoder = 0;

        x = 0;
        y = 0;
    }
    public void updatePosition() {
        double rightBackEncoder = getRightBackDriveEncoder();
        double leftBackEncoder = getLeftBackDriveEncoder();
        double chinEncoder = getChinMotorEncoder();
        //double dom1Encoder = getDOM1MotorEncoder();
        //double dom2Encoder = getDOM2MotorEncoder();
        //double domExtendEncoder = getDOMMotorExtendEncoder();

        //double averageChange = (leftBackEncoder - lastLeftBackEncoder);
        double averageChange = ((leftBackEncoder - lastLeftBackEncoder) + (rightBackEncoder - lastRightBackEncoder))/2.0;


        if (isGoldFound && lastIsGoldFound) {
            twoCyclesIsGoldFound = true;
        } else {
            twoCyclesIsGoldFound = false;
        }

        if (isGoldAligned && lastIsGoldAligned) {
            twoCyclesIsGoldAligned = true;
        } else {
            twoCyclesIsGoldAligned = false;
        }

        if (isGoldCentered && lastIsGoldCentered) {
            twoCyclesIsGoldCentered = true;
        } else {
            twoCyclesIsGoldCentered = false;
        }

        //gravity  = imu.getGravity();
        //getIMUHeading();
        //getEulerAngles(eulerAngles);;

        //heading from gyro
        //double heading = Math.toRadians(getHeading());
        //heading from imu
        heading = Math.toRadians(getIMUHeading());
        heading1 = Math.toRadians(getIMU1Heading());
        x += averageChange * Math.cos(heading);
        y += averageChange * Math.sin(heading);
        lastRightBackEncoder = rightBackEncoder;
        lastLeftBackEncoder = leftBackEncoder;
        lastChinEncoder = chinEncoder;
        //lastDOM1Encoder = dom1Encoder;
        //lastDOM2Encoder = dom2Encoder;
        //lastDOMExtendEncoder = domExtendEncoder;

        lastIsGoldFound = isGoldFound;
        lastIsGoldAligned = isGoldAligned;
        lastIsGoldCentered = isGoldCentered;

        //lastRightBackEncoder = rightBackEncoder;
        //lastLeftBackEncoder = leftBackEncoder;
        //lastRightFrontEncoder = rightFrontEncoder;
        //lastLeftFrontEncoder = leftFrontEncoder;
        //telemetry.addData("Gyro Heading:", gyro.getHeading());
        //telemetry.addData("Gyro IO Heading: ", getHeading());

        //telemetry.addData("IMU Heading:", angles.firstAngle);
        //telemetry.addData("IMU IO Heading: ", Math.toDegrees(heading));

        /*telemetry.addData("Distance left sensor (cm)",
                String.format(Locale.US, "%.02f", leftDistance.getDistance(DistanceUnit.CM)));

        telemetry.addData("Distance right sensor(cm)",
                String.format(Locale.US, "%.02f", rightDistance.getDistance(DistanceUnit.CM)));*/

        telemetry.addData("IMU Heading",  "Starting at %.2f",
                getIMUHeading());
        telemetry.addData("IMU1 Heading",  "Starting at %.2f",
                getIMU1Heading());

/*        telemetry.addData("IMU Y",  "Starting at %.2f",
                angles.secondAngle);
        telemetry.addData("IMU1 Y",  "Starting at %.2f",
                angles.secondAngle);

        telemetry.addData("IMU X",  "Starting at %.2f",
                angles.thirdAngle);
        telemetry.addData("IMU1 X",  "Starting at %.2f",
                angles.thirdAngle);*/

    }

    public double getX() {
        return x / COUNTSPERINCH;
    }
    public double getY() {
        return y / COUNTSPERINCH;
    }

/*    public double getHeading() {
        double gyroheading = gyro.getHeading() - gyroOffset;
        while (gyroheading > 180) {
            gyroheading -= 360;
        }
        while (gyroheading <= - 180) {
            gyroheading += 360;
        }
        return -gyroheading;
    }*/

    public double getIMUHeading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double imuHeading = angles.firstAngle - imuOffset;
        while (imuHeading > 180) {
            imuHeading -= 360;
        }
        while (imuHeading <= - 180) {
            imuHeading += 360;
        }
        return -imuHeading;
    }

    public double getIMU1Heading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles1   = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double imu1Heading = angles1.firstAngle - imu1Offset;
        while (imu1Heading > 180) {
            imu1Heading -= 360;
        }
        while (imu1Heading <= - 180) {
            imu1Heading += 360;
        }
        return -imu1Heading;
    }

    /**
     * This method returns the Euler angles of all 3 axes from quaternion orientation.
     *
     * @param angles specifies the array to hold the angles of the 3 axes.
     */
    private void getEulerAngles(double[] angles)
    {
        Quaternion q = imu.getQuaternionOrientation();

        //
        // 0: roll (x-axis rotation)
        // 1: pitch (y-axis rotation)
        // 2: yaw (z-axis rotation)
        //
        angles[0] = Math.toDegrees(Math.atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y)));
        double sinp = 2.0*(q.w*q.y - q.z*q.x);
        angles[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
        angles[2] = Math.toDegrees(Math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)));
    }   //getEulerAngles

    private void getEulerAngles1(double[] angles1)
    {
        Quaternion q1 = imu1.getQuaternionOrientation();

        //
        // 0: roll (x-axis rotation)
        // 1: pitch (y-axis rotation)
        // 2: yaw (z-axis rotation)
        //
        angles1[0] = Math.toDegrees(Math.atan2(2.0*(q1.w*q1.x + q1.y*q1.z), 1.0 - 2.0*(q1.x*q1.x + q1.y*q1.y)));
        double sinp = 2.0*(q1.w*q1.y - q1.z*q1.x);
        angles1[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
        angles1[2] = Math.toDegrees(Math.atan2(2.0*(q1.w*q1.z + q1.x*q1.y), 1.0 - 2.0*(q1.y*q1.y + q1.z*q1.z)));
    }

    public double getRightBackDriveEncoder() {
        return rightBackDrive.getCurrentPosition() - rightBackOffset;
    }

    public double getLeftBackDriveEncoder() {
        return leftBackDrive.getCurrentPosition() - leftBackOffset;
    }

    public double getChinMotorEncoder() {
        return chinMotor.getCurrentPosition() - chinOffset;
    }

    /*public double getDOM1MotorEncoder() {
        return dom1Motor.getCurrentPosition() - dom1Offset;
    }

    public double getDOM2MotorEncoder() {
        return dom2Motor.getCurrentPosition() - dom2Offset;
    }

    public double getDOMMotorExtendEncoder() {
        return domExtendMotor.getCurrentPosition() - domExtendOffset;
    }*/

    public double getDOMPotVoltage() {
        return domPot.getVoltage();
    }

    public double getDOMPotDegrees() {
        return (270 - (domPot.getVoltage()*DEGREESPERVOLT));
    }

    public double getGoldXPosition() {
        return (goldXPosition);
    }
    public double getGoldXPositionAroundZero() {
        return (goldXPosition - 320);
    }


    public void setDrivePower(double left, double right) {
        rightBackDrive.setPower(right);
        leftBackDrive.setPower(left);
    }

    /*public double getLightReading(){
        return ods.getLightDetected();
    }*/

    public void calibrateGyroandIMU() {
        // start calibrating the gyro.
        telemetry.addData(">", "IMU Calibrating. Do Not Move!");
        //telemetry.update();

        sleep(500);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        //gyro.calibrate();
        imu.initialize(parameters);
    }

    public void calibrateGyroandIMU1() {
        // start calibrating the gyro.
        telemetry.addData(">", "IMU1 Calibrating. Do Not Move!");
        //telemetry.update();

        sleep(500);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        //gyro.calibrate();
        imu1.initialize(parameters);
    }

    /*public void setJewelColor(int jcolor){
        jewelColor = jcolor;
    }
    public int getJewelColor(){
        return jewelColor;
    }

    public void setLeftProximityAverage(double leftproximityaverage){
        leftProximityAverage = leftproximityaverage;
    }
    public double getLeftProximityAverage(){
        return leftProximityAverage;
    }

    public void setRightProximityAverage(double rightproximityaverage){
        rightProximityAverage = rightproximityaverage;
    }
    public double getRightProximityAverage(){
        return rightProximityAverage;
    }

    public double getProximityCorrection(){
        if (leftProximityAverage > 20){
            //proximityCorrection = 20;
            proximityCorrection = 5;
        } else if((leftProximityAverage >= 7) && (leftProximityAverage <= 11)) {
            proximityCorrection = 0;
        } else if(leftProximityAverage > 11) {
            //proximityCorrection = (((20 * leftProximityAverage) / 13) - (140/13));
            proximityCorrection = (((5.0 * leftProximityAverage) / 9.0) - (55.0/9.0));
        } else if (rightProximityAverage > 20) {
            //proximityCorrection = -20;
            proximityCorrection = -5;
        } else if((rightProximityAverage >= 7) && (rightProximityAverage <= 11)) {
            proximityCorrection = 0;
        } else if (rightProximityAverage > 11) {
            //proximityCorrection = -(((20 * rightProximityAverage) / 13) - (140 / 13));
            proximityCorrection = -(((5.0 * rightProximityAverage) / 9.0) - (55.0/9.0));
        } else {
            proximityCorrection = 0;
        }
        return -proximityCorrection;
    }*/

    public void setAllianceColor(int acolor){
        allianceColor = acolor;
    }
    public int getAllianceColor(){
        return allianceColor;
    }

    /*public void setVuMark(int location){
        vuMark = location;
    }
    public int getVuMark(){
        return vuMark;
    }*/

    /*public void setHook(double open_close) {
        leftHand.setPosition(open_close);
        rightHand.setPosition(1 - open_close);
    }

    public void setRelicHand(double position) {
        relicHand.setPosition(1 - position);
    }

    public void setJewelArm(double position) {
        jewelArm.setPosition(1 - position);
    }

    public void setProximityArm(double position) {
        proximityArm.setPosition(1 - position);
    }*/

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
