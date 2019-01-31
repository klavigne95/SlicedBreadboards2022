package org.firstinspires.ftc.teamcode.teamcode2017;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.CENTER;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.LEFT;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.RIGHT;

@TeleOp(name = "AltAutonomousGyro", group = "Auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AlternateAutoGyro extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(true, StartPosition.marker);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        initDetector();

        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            robot.composeTelemetry();
            // UNHOOK //
            if(robot.isHooked) {
                robot.liftMotor.setPower(-0.75);
                wait1(250);
                robot.pulleyHolder.setPosition(.655f); //latch is .168
                wait1(1500);
                robot.liftMotor.setPower(1);
                wait1(1500);
                robot.gyrodrive.vertical(0.7, Convert.tileToYeet(-.75), robot.getHeading());
                robot.gyrodrive.vertical(0.7, Convert.tileToYeet(.05), robot.getHeading());
                robot.gyrodrive.horizontal(0.7, Convert.tileToYeet3(-0.207), robot.getHeading());
                robot.liftMotor.setPower(-0.5);
                robot.gyrodrive.vertical(0.7, Convert.tileToYeet(-.25), robot.getHeading());
                robot.gyrodrive.horizontal(0.7, Convert.tileToYeet3(.207), robot.getHeading());
            }
            robot.gyrodrive.newGyroTurn(0.7, 180);
            robot.gyrodrive.vertical(0.7, Convert.tileToYeet(-0.207), 180);


            // SCAN GLYPHS //
            SamplingOrderDetector.GoldLocation glyphPosition;
            if(detector.isFound()){
                if (detector.getXPosition() <= 210 && detector.getXPosition() > 0){
                    glyphPosition = LEFT;
                    telemetry.addData("Glyph Position:", glyphPosition + " " +detector.getXPosition());
                } else if (detector.getXPosition() > 210 && detector.getXPosition() <= 530){
                    glyphPosition = CENTER;
                    telemetry.addData("Glyph Position:", glyphPosition + " " +detector.getXPosition());
                } else if (detector.getXPosition() > 530 && detector.getXPosition() <= 640){
                    glyphPosition = RIGHT;
                    telemetry.addData("Glyph Position:", glyphPosition + " " +detector.getXPosition());
                } else {
                    glyphPosition = RIGHT;
                    telemetry.addData("OUT OF BOUNDS, Default RIGHT", glyphPosition);
                }
            } else {
                // TODO: Move robot to find glyphs
                glyphPosition =  RIGHT;
                telemetry.addData("NO DETECTOR, Default RIGHT", glyphPosition);
            }
            telemetry.addData("Glyph Position: ", glyphPosition);
            telemetry.update();

            // IF POINTED TO DEPOT //
            if(robot.startPosition == StartPosition.marker){
                // Move Gold
                int angleToMineral;
                int angleToMarker;
                int angleToCrater;
                int distToMineral;
                int distToMarker;
                if (glyphPosition == LEFT) {
                    angleToMineral = 160; //180 - 30
                    angleToMarker = 212; //160 +52
                    angleToCrater = 238; // 212 +26
                    distToMineral = Convert.tileToYeet(1.633);
                    distToMarker = Convert.tileToYeet(1.633); //1.5 but not far enough
                } else if (glyphPosition == CENTER){
                    angleToMineral = 179;
                    angleToMarker = 179;
                    angleToCrater = 225;
                    distToMineral = Convert.tileToYeet(1.414);
                    distToMarker = Convert.tileToYeet(1.414);
                } else if (glyphPosition == RIGHT){
                    angleToMineral = 160;
                    angleToMarker = 108; //160-52
                    angleToCrater = 172; //108+64
                    distToMineral = Convert.tileToYeet(1.633);
                    distToMarker = Convert.tileToYeet(1.5);
                } else {
                    angleToMineral = 160;
                    angleToMarker = 108; //160-52
                    angleToCrater = 172; //108+64
                    distToMineral = Convert.tileToYeet(1.633);
                    distToMarker = Convert.tileToYeet(1.5);
                    telemetry.addData("GLYPH POSITION NOT DEFINED, Defualting", glyphPosition);
                }
                telemetry.addData("Turning, angleToMineral: ",  angleToMineral);
                telemetry.addData("Moving, distToMineral: " , distToMineral);
                telemetry.addData("Turning, angleToMarker: " , angleToMarker);
                telemetry.addData("Moving, distToMarker: " , distToMarker);
                telemetry.update();

                robot.gyrodrive.newGyroTurn(0.7, angleToMineral);
                robot.gyrodrive.vertical(-0.7, distToMineral, robot.getHeading());
                robot.gyrodrive.newGyroTurn(0.7, angleToMarker);
                robot.gyrodrive.vertical(-0.7, distToMarker, robot.getHeading());

                // Set Marker
                telemetry.addData("Currently: ", "DEPLOYING MARKER");
                telemetry.update();
                deployMarker();
                // Park in Crater
                telemetry.addData("Turning, angleToCrater: " , angleToCrater);
                telemetry.addData("Moving, distToCrater: " , Convert.tileToYeet(4.5));
                telemetry.update();

                robot.gyrodrive.newGyroTurn(0.7, angleToCrater);

                robot.drive.resetMotors();
                robot.drive.vertical(Convert.tileToYeet2(-4.5));            } else if (robot.startPosition == StartPosition.crater){
                telemetry.addData("Glyph Position: ", glyphPosition);
                telemetry.update();
                // Set Marker
                robot.gyrodrive.newGyroTurn(0.7, 135); //180 - 45
                robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(2), robot.getHeading());
                robot.gyrodrive.newGyroTurn(0.7, 0); // 135 -135
                robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(3.535), robot.getHeading());
                robot.gyrodrive.newGyroTurn(0.7, 135); //0 + 135
                robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(3), robot.getHeading());
                // Park in Crater, While Moving Gold
                robot.gyrodrive.newGyroTurn(0.7, 45); //135 - 90
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(-3), robot.getHeading());
                wait1(1000);
                robot.gyrodrive.newGyroTurn(0.7, 90); //45 + 45
                if (glyphPosition == LEFT) {
                    robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(-1.414), robot.getHeading());
                } else if (glyphPosition == CENTER) {
                    robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(-2.121), robot.getHeading());
                } else {
                    robot.gyrodrive.vertical(-0.7, Convert.tileToYeet(-2.828), robot.getHeading());
                    telemetry.addData("Glyph position may or may not be defined", glyphPosition);
                }
                robot.gyrodrive.newGyroTurn(0.7, 0); // 90 - 90

                robot.drive.resetMotors();
                robot.drive.vertical(Convert.tileToYeet2(-2));
            }
            wait1(1000000000);
        }
    }

    public void deployMarker() throws InterruptedException{
        robot.markerServo.setPosition(.487f);
        wait1(1000);
        robot.markerServo.setPosition(.637f);
        wait1(250);
    }

    private void inputGameConfig() {
        telemetry.addData("Input which side", "Left (Square) or right (Crater) (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.marker;
        } else {
            robot.startPosition = StartPosition.crater;
        }
        telemetry.addData("Chosen Start Position", robot.startPosition);

        telemetry.addData("Are you starting Hooked?", "Yes (Y) or No (X)");
        telemetry.update();
        while (!gamepad1.y && !gamepad1.x) {
        }

        if (gamepad1.y) {
            robot.isHooked = true;
        } else {
            robot.isHooked = false;
        }

        telemetry.addData("isHooked?", robot.isHooked);
        telemetry.addData("Start postion", robot.startPosition);
        telemetry.update();
    }

    public void wait1(int t) throws InterruptedException {
        TimeUnit.MILLISECONDS.sleep(t);
    }

    public android.hardware.Camera initVision() {
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

    public void initDetector(){
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }
}
