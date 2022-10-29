/*
    Created by 23spatel on 9/16/22
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class Drivetrain {
    
    // declare hardware variables
    private HardwareMap hwMap;
    private DcMotor frmotor;
    private DcMotor flmotor;
    private DcMotor brmotor;
    private DcMotor blmotor;
    
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    
    private double y, x, rotX, botHeading, rotY, denominator;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    public Drivetrain(HardwareMap hwMap){
        init(hwMap);
        resetMotors();
        stop();
    }
    
    private void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        hwMap.logDevices();
    }
    
    private void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        frmotor = hwMap.dcMotor.get("frontR");
        flmotor = hwMap.dcMotor.get("frontL");
        brmotor = hwMap.dcMotor.get("backR");
        blmotor = hwMap.dcMotor.get("backL");

    }
    
    private void stop() {
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);
        resetMotors();
    }
    
    public void resetMotors() {
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            
    }
    
    public void resetMotors(boolean val){
        resetMotors();
        
        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    
    public void useJoystick(double y, double x, double rx, double ry, double heading){

        // Read reverse IMU heading, as the IMU heading is CW positive
        botHeading = -1*heading;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
        move(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    
    public void useTrigger(double left, double right){
        if (right > 0){move(right* -1);}
            else if (left > 0){move(left);}
            
    }
    
    public void moveFwd(double dist, double pwr){
        resetMotors(true);
        double clicksPerCm = 27.5;
        int flPos, frPos, blPos, brPos; 
        
        flPos = (flmotor.getCurrentPosition() + (int) ((dist * clicksPerCm)+0.5));
        frPos = (frmotor.getCurrentPosition() + (int) ((dist * clicksPerCm)+0.5));
        blPos = (blmotor.getCurrentPosition() + (int) ((dist * clicksPerCm)+0.5));
        brPos = (frmotor.getCurrentPosition() + (int) ((dist * clicksPerCm)+0.5));
        
        flmotor.setTargetPosition(flPos);
        frmotor.setTargetPosition(frPos);
        blmotor.setTargetPosition(blPos);
        brmotor.setTargetPosition(brPos);
        
        move(pwr);
        
        while (flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()){}
        
        move(0);
    }
    
    
    private void move(double flpower, double frpower, double blpower, double brpower){
        flmotor.setPower(flpower);
        frmotor.setPower(frpower);
        blmotor.setPower(blpower);
        brmotor.setPower(brpower);
    }
    
    private void move(double power){
        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);
    }
    
    
    public void rotate(double degrees, double pwr) //doesn't work
    {
        resetMotors(true);
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        
        
        if (degrees < 0)
        {
           while (degrees >= imu.getAngularOrientation().firstAngle) 
           {
               double clicksPerCm = 27.5;
                int flPos, frPos, blPos, brPos; 
        
                flPos = (flmotor.getCurrentPosition() + (int) ((1 * clicksPerCm)+0.5));
                frPos = (frmotor.getCurrentPosition() + (int) ((-1 * clicksPerCm)+0.5));
                blPos = (blmotor.getCurrentPosition() + (int) ((1 * clicksPerCm)+0.5));
                brPos = (frmotor.getCurrentPosition() + (int) ((-1 * clicksPerCm)+0.5));
        
        
                move(pwr); 
                
                flmotor.setTargetPosition(flPos);
                frmotor.setTargetPosition(frPos);
                blmotor.setTargetPosition(blPos);
                brmotor.setTargetPosition(brPos);
        
                
                
                while (flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()){}
           }
        
            move(0);
        }
        else
        {
            while (degrees <= imu.getAngularOrientation().firstAngle)
            {
               double clicksPerCm = 27.5;
                int flPos, frPos, blPos, brPos; 
        
                flPos = (flmotor.getCurrentPosition() + (int) ((-10 * clicksPerCm)+0.5));
                frPos = (frmotor.getCurrentPosition() + (int) ((10 * clicksPerCm)+0.5));
                blPos = (blmotor.getCurrentPosition() + (int) ((-10 * clicksPerCm)+0.5));
                brPos = (frmotor.getCurrentPosition() + (int) ((10 * clicksPerCm)+0.5));
        
        
                move(pwr);
                
                flmotor.setTargetPosition(flPos);
                frmotor.setTargetPosition(frPos);
                blmotor.setTargetPosition(blPos);
                brmotor.setTargetPosition(brPos);
        
               
                
                while (flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()){}
            }
        
            move(0);
        }
        
    }
    
    
    
    
    
}
