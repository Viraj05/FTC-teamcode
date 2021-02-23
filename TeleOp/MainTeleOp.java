package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "LinearOpMode")
public class TeleOp_Code extends LinearOpMode
{

    public DcMotor A;
    public DcMotor B;
    public DcMotor Shooter1;

    public void GamePad_1()
    {
        double Linear ;
        double Turn ;

        Linear = -gamepad1.left_stick_y;

        Turn = -gamepad1.left_stick_x;

        A.setPower(-Linear+Turn);
        B.setPower(Linear+Turn);

        while(gamepad1.dpad_up)
        {

            A.setPower(-0.5);
            B.setPower(0.5);
        }

        while(gamepad1.dpad_down)
        {

            A.setPower(0.5);
            B.setPower(-0.5);
        }

        while(gamepad1.dpad_left)
        {

            A.setPower(0);
            B.setPower(0.5);
        }

        while(gamepad1.dpad_right)
        {

            A.setPower(0.5);
            B.setPower(0);
        }

        while(gamepad1.y)
        {

            A.setPower(-0.2);
            B.setPower(0.2);
        }

        while(gamepad1.a)
        {

            A.setPower(0.2);
            B.setPower(-0.2);
        }

        while(gamepad1.x)
        {

            A.setPower(0);
            B.setPower(0.2);
        }

        while(gamepad1.b)
        {

            A.setPower(-0.2);
            B.setPower(0);
        }

    }
    private void Shootera(double power) {
        Shooter1.setPower(power);
    }
    private void Shooterb(double power) {
        Shooter1.setPower(-power);
    }


    @Override
    public void runOpMode()
    {
        //A=hardwareMap.dcMotor.get("A");
        //B=hardwareMap.dcMotor.get("B");
        Shooter1=hardwareMap.dcMotor.get("Shooter1");

        waitForStart();
        while(opModeIsActive())
        {
            //GamePad_1();
            Shootera(gamepad1.right_trigger);
            Shooterb(gamepad1.left_trigger);
            //telemetry.addData("Value",Shooter1.getPortNumber());
            //telemetry.addData("number", gamepad1.left_bumper);
            //telemetry.addData("number", gamepad1.a);
            //telemetry.update();

        }
    }
}
