package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

/**
 * Created by 20smcnamara on 10/19/18.
 */
@TeleOp(name = "Yeet", group = "Test")
public class yeet extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        // Initialization
        robot = new Robot2017(false, StartPosition.marker);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        // Running
        waitForStart();
        while (opModeIsActive()) {
            robot.gyrodrive.horizontal(0.7, 10, robot.getHeading());

            TimeUnit.MILLISECONDS.sleep(1000000000);
            idle(); // idle sometimes is weird soooo wait ^
        }
    }

}