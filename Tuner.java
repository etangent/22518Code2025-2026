package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Tuner {

    public static class Param {
        public String name;
        public double value;
        public double increment;

        public Param(String name, double value, double increment) {
            this.name = name;
            this.value = value;
            this.increment = increment;
        }
    }

    private final Param[] params;
    private int index = 0;

    private boolean prevLeft, prevRight, prevUp, prevDown, prevX, prevY;

    public Tuner(Param... params) {
        this.params = params;
    }

    public void update(Gamepad gamepad) {
        boolean left = gamepad.dpad_left && !prevLeft;
        boolean right = gamepad.dpad_right && !prevRight;
        boolean up = gamepad.dpad_up && !prevUp;
        boolean down = gamepad.dpad_down && !prevDown;
        boolean x = gamepad.x && !prevX;
        boolean y = gamepad.y && !prevY;

        if (right) {
            index = (index + 1) % params.length;
        }
        if (left) {
            index = (index - 1 + params.length) % params.length;
        }

        Param p = params[index];

        if (up) p.value += p.increment;
        if (down) p.value -= p.increment;

        if (x) p.increment *= 10.0;
        if (y) p.increment /= 10.0;

        prevLeft = gamepad.dpad_left;
        prevRight = gamepad.dpad_right;
        prevUp = gamepad.dpad_up;
        prevDown = gamepad.dpad_down;
        prevX = gamepad.x;
        prevY = gamepad.y;
    }

    public Param getSelected() {
        return params[index];
    }

    public double get(String name) {
        for (Param p : params)
            if (p.name.equals(name))
                return p.value;
        return 0;
    }

    public Param[] getParams() {
        return params;
    }
}
