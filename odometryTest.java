package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Math.*;

@TeleOp(name = "odometryTest (Java)")
public class odometryTest extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    
    
    private Pose2D pos;
    private Pose2D alignPose = new Pose2D(DistanceUnit.MM, 658.25, -739.7, AngleUnit.RADIANS, -2.414);
    
    Tuner tuner = new Tuner(
      new Tuner.Param("strafeRatio", 1.2, .001),
      new Tuner.Param("driveKp", .007, .001),
      new Tuner.Param("driveKd", .001, .001),
      new Tuner.Param("turnKp", 1, .001),
      new Tuner.Param("turnKd", .0, .001)  
    );
    
    double x;
    double y;
    double rx;

    // just logging align values
    double Vx = 0;
    double Vy = 0;
    
    GoBildaPinpointDriver odo;
    
    double time = 0;
    
    
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        
        
        
        odo.setOffsets(12, -156, DistanceUnit.MM);
        
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        
        odo.resetPosAndIMU();
        
        waitForStart();
        resetRuntime();
        
        while (opModeIsActive()) {
            odo.update();
            pos = odo.getPosition();
            tuner.update(gamepad1);
            
            if (gamepad1.b) {
                odo.resetPosAndIMU();
            } else if (!gamepad1.a) {
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                drive();
            } else {
                align(alignPose);
            }
            
            double newTime = getRuntime();
            telemetry.addData("Loop time", newTime - time);
            time = newTime;
            

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Position", data);
            telemetry.addLine("\n=== Tuner ===");
            for (Tuner.Param param : tuner.getParams()) {
                if (param == tuner.getSelected())
                    telemetry.addData("> " + param.name, "%.4f  (inc=%.4f)", param.value, param.increment);
                else
                    telemetry.addData("  " + param.name, "%.4f", param.value);
            }
            String vector = String.format(Locale.US, "{Vx: %.3f, Vy: %.3f}", Vx, Vy);
            telemetry.addData("Displacement vector", vector);
            telemetry.update();
        }
    }

    public void align(Pose2D target) {
        double heading = pos.getHeading(AngleUnit.RADIANS);
        // field relative: x is forward, y is to the right
        double yError = (target.getY(DistanceUnit.MM) - pos.getY(DistanceUnit.MM));
        double xError = (target.getX(DistanceUnit.MM) - pos.getX(DistanceUnit.MM));
        double turnError = AngleUnit.normalizeRadians(target.getHeading(AngleUnit.RADIANS) - heading);

        double distance = hypot(xError, yError);
        double errorVel = (xError * odo.getVelX(DistanceUnit.MM) + yError * odo.getVelY(DistanceUnit.MM)) / distance;

        x = 0;
        y = 0;
        rx = 0;
        
        if (!atHeading(target.getHeading(AngleUnit.RADIANS))) {
            rx = -1 * tuner.get("turnKp") * turnError + tuner.get("turnKd") * odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        } else if (distance > 10) {
            double setpointScale = Math.min(tuner.get("driveKp") * distance - tuner.get("driveKd") * errorVel, 1 / tuner.get("strafeRatio"));
            Vx = xError / distance * setpointScale;
            Vy = yError / distance * setpointScale;
            // now converting to robot relative, y is forward
            y = Vx * cos(heading) + Vy * sin(heading);
            x = -1 * tuner.get("strafeRatio") * (-Vx * sin(heading) + Vy * cos(heading));
        }
        
        drive();
    }
    
    private boolean atHeading(double heading) {
        return Math.abs(AngleUnit.normalizeRadians(heading - pos.getHeading(AngleUnit.RADIANS))) < 0.1;
    }
    
    private void drive() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FrontLeft.setPower((y + x + rx)/denominator);
        FrontRight.setPower(((y - x) - rx)/denominator);
        BackLeft.setPower(((y - x) + rx)/denominator);
        BackRight.setPower(((y + x) - rx)/denominator);
    }
}
