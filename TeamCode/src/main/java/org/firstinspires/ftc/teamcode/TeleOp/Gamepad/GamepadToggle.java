package org.firstinspires.ftc.teamcode.TeleOp.Gamepad;

import com.qualcomm.robotcore.util.ElapsedTime;


public class GamepadToggle{

    GamepadWrapper wrapper;
    GamepadButton button;
    private boolean value = false;

    private ElapsedTime timer = new ElapsedTime();

    GamepadToggle(GamepadWrapper wrapper, GamepadButton button) {
        this.wrapper = wrapper;
        this.button = button;
        timer.reset();
    }

    public boolean getValue() {
        return value;
    }

    public void update() {
            boolean currentValue = wrapper.buttonState(button);

            if (timer.seconds() > .25 && currentValue){
                value = !value;
                timer.reset();
            }
    }

}
