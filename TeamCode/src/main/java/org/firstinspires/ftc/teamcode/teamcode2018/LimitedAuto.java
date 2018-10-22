package org.firstinspires.ftc.teamcode.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "LimitAutonomous", group = "red")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class LimitedAuto extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.marker);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();
        while (opModeIsActive()) {
            int RE_ADJUST = 0;
            int POINT_TOWARDS_CRATER = 0;
            int ANGLE_PARALLEL_CRATER = 0;
            String glyphPosition = "left";

            // Get Down
            robot.lift.setPower(.5);
            wait1(500);
            robot.lift.setPower(0);
            robot.drive.vertical(.1);
            robot.lift.setPower(-.5);
            wait1(500);
            robot.lift.setPower(0);

            // If Pointed at Square ->
            if(robot.startPosition == StartPosition.marker && robot.teamColor == TeamColor.red){
                // Scan Glyphs

                // Move Gold
                int angleToMineral;
                int angleToMarker;
                if (glyphPosition == "left") {
                    angleToMineral = -30;
                    angleToMarker = 52;
                } else if (glyphPosition == "middle"){
                    angleToMineral = 0;
                    angleToMarker = 0;
                } else if (glyphPosition == "right"){
                    angleToMineral = 30;
                    angleToMarker = -52;
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(10);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(10);
                // Set Marker
                robot.drive.vertical(10);
                deployMarker();
                // Park in Crater
                //NOT IN LIMITED
            } else if (robot.startPosition == StartPosition.marker && robot.teamColor == TeamColor.blue){
                // Scan Glyphs

                // Move Gold
                int angleToMineral;
                int angleToMarker;
                if (glyphPosition == "left") {
                    angleToMineral = -30;
                    angleToMarker = 52;
                } else if (glyphPosition == "middle"){
                    angleToMineral = 0;
                    angleToMarker = 0;
                } else if (glyphPosition == "right"){
                    angleToMineral = 30;
                    angleToMarker = -52;
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(10);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(10);
                // Set Marker
                robot.drive.vertical(10);
                deployMarker();
                // Park in Crater
                //NOT IN LIMITED
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.red){
                //Scan Glyphs

                // Set Marker NOT IN LIMITED
                int angleToMineral;
                int angleToCrater;
                // Park in Crater, While Moving Gold
                if(glyphPosition == "left"){
                    angleToMineral = -30;
                    angleToCrater = 30;
                } else if (glyphPosition == "middle"){
                    angleToMineral = 0;
                    angleToCrater = 0;
                } else if (glyphPosition == "right") {
                    angleToMineral = 30;
                    angleToCrater = -15;
                } else {
                    angleToMineral = 0;
                    angleToCrater = 0;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(10);
                robot.drive.turn(angleToCrater);
                robot.drive.turn(10);
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.blue){
                //Scan Glyphs

                // Set Marker NOT IN LIMITED
                int angleToMineral;
                int angleToCrater;
                // Park in Crater, While Moving Gold
                if(glyphPosition == "left"){
                    angleToMineral = -30;
                    angleToCrater = 30;
                } else if (glyphPosition == "middle"){
                    angleToMineral = 0;
                    angleToCrater = 0;
                } else if (glyphPosition == "right") {
                    angleToMineral = 30;
                    angleToCrater = -15;
                } else {
                    angleToMineral = 0;
                    angleToCrater = 0;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(10);
                robot.drive.turn(angleToCrater);
                robot.drive.turn(10);
            }
        }
    }

    public void deployMarker() throws InterruptedException{
        robot.markerServo.setPosition(.90f);
        wait1(1000);
        robot.markerServo.setPosition(.20f);
        wait1(1000);
    }

    private void inputGameConfig() throws InterruptedException {
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);

        telemetry.addData("Input which side", "Left (Square) or right (Crater) (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.marker;
        } else {
            robot.startPosition = StartPosition.crater;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
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

}
