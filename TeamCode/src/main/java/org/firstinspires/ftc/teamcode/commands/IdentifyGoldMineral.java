package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.teamcode.utilities.IO_RoverRuckus_Test;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import org.firstinspires.ftc.teamcode.utilities.IO_RoverRuckus_Test;

/**
 * Created by HPaul on 10/25/2017.
 */

public class IdentifyGoldMineral extends BasicCommand{

    // Setup variables
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    // DogeCV detector
    GoldAlignDetector detector;

    long timeOut;

    public IdentifyGoldMineral(){
    }

    public void init() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        timeOut = System.currentTimeMillis() + 10000;
        // Default webcam name
        webcamName = io.webcamName;
        // Set up parameters for Vuforia
        int cameraMonitorViewId = io.cameraMonitorViewId;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = "Ae+uGTX/////AAABmQV3De8djUCDjn2zDZDbCssvJv8/irA8Dzm+UnPYeGcgN7Y/V1EFU/DgmBcA3x5TxqeooD4B02M6PR+5IBifNlYVIXezFdgl/f9PKHDE7KAl3yeEV993njRk8ocjpNJwYDqcN1vZP6yWRqe4Y9QdAJH+KZPQeR+eN5wT87m4ZNHhsC5DidIkFYuhVNVdM+Gn9CLUphmjX1woXqSLqK3BdmU6XEfKU730USi7clKwVidBUMCcFcL878gUG0Mn5JL7dcPUO3r1q+8ODt1wInwPWgSQlXrrY4wWSeHJ5VwwihGnisIZ2Ps41yqf1QtrzK7FsDz5P5aQaQ7rVtzntFLZZ+ftIy0aJ+YelBy1QtZX+dc8";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        //Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate the targets
        targetsRoverRuckus.activate();

        // Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(map.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter


        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        //detector.downscale = 0.4; // How much to downscale the input frames

        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.useDefaults(); // Use default settings
        detector.perfectAreaScorer.perfectArea = 1369; // if using PERFECT_AREA scoring
        //detector.maxAreaScorer.weight = 0.005; //
        //detector.maxAreaScorer.weight = 3; //
        detector.perfectAreaScorer.weight = 10;

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //detector.useDefaults();
        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        //detector.downscale = 0.8;
        detector.downscale = 0.8;









        /*detector.useDefaults(); // Use default settings


        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment



        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //detector.useDefaults();
        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring

        //detector.downscale = 0.8;*/

        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }

    public void execute(){
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
    }

    public boolean isFinished(){
        /*if (vuMark == null) {
            //io.setVuMark(IO_4WD_Test.UNKNOWN);
            return false;
        } else {*/
            //io.setVuMark(vuMark.ordinal());
            //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
            //telemetry.addData("VuMark from IdentifyVuMark from IO", "%s visible", vuMark);
            /*if (vuMark == RelicRecoveryVuMark.LEFT ) {
                io.setVuMark(IO_4WD_Test.LEFT);
                return true;
            } else if (vuMark == RelicRecoveryVuMark.CENTER){
                io.setVuMark(IO_4WD_Test.CENTER);
                return true;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                io.setVuMark(IO_4WD_Test.RIGHT);
                return true;
            } else {
                telemetry.addData("vuMark", "Unknown");
                telemetry.addData("vuMark", "not identified");
                //io.setVuMark(IO_4WD_Test.UNKNOWN);
                return false;
            }*/
            //return vuMark == (RelicRecoveryVuMark.LEFT || RelicRecoveryVuMark.RIGHT || RelicRecoveryVuMark.CENTER) || System.currentTimeMillis() >= timeOut;
            //return vuMark != RelicRecoveryVuMark.UNKNOWN || System.currentTimeMillis() >= timeOut;
        //}

        if (io.getDOMPotDegrees() >=15) {
            if (detector.isFound()) {
                io.isGoldFound = true;
                io.goldXPosition = detector.getXPosition();
            } else {
                io.isGoldFound = false;
                io.goldXPosition = 0;
            }

            if (detector.getAligned()) {
                io.isGoldAligned = true;
            } else {
                io.isGoldAligned = false;
            }

        }

        /*telemetry.addData("IsFound" , detector.isFound()); // Is the gold mineral found?
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        telemetry.addData("Gold Position Around Zero: ", io.getGoldXPositionAroundZero());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Completed Search: ", io.completedSearch );
        telemetry.addData("Captured Gold Heading: ", io.headingOfGold);
        telemetry.addData("Two Cycles Gold Found: ", io.twoCyclesIsGoldFound);
        telemetry.addData("Two Cycles Gold Aligned: ", io.twoCyclesIsGoldAligned);
        telemetry.addData("Two Cycles Gold Centered: ", io.twoCyclesIsGoldCentered);
        telemetry.addData("Gold Position: ", io.getGoldXPosition());
        telemetry.addData("Is Gold Found: ", io.isGoldFound);
        telemetry.addData("Is Gold Aligned: ", io.isGoldAligned);*/
        telemetry.addData("Mode:", "Identify Gold Mineral");
        return io.twoCyclesIsGoldCentered || System.currentTimeMillis() >= timeOut;
        //return System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        detector.disable();
        vuforia.stop();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
