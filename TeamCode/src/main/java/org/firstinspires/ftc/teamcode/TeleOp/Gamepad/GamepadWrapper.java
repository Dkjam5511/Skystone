package org.firstinspires.ftc.teamcode.TeleOp.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {

    private Gamepad gamepad;

    GamepadToggle rightBumberToggle;
    GamepadToggle leftBumberToggle;


    public GamepadWrapper(Gamepad gamepad) {
        this.gamepad = gamepad;

        rightBumberToggle = new GamepadToggle(this, GamepadButton.RIGHT_BUMPER);
        leftBumberToggle = new GamepadToggle(this, GamepadButton.LEFT_BUMPER);

    }

    public void update(){
        rightBumberToggle.update();
        leftBumberToggle.update();
    }

    public boolean getLeftBumber() {
        return leftBumberToggle.getValue();
    }

    public boolean getRightBumber() {
        return rightBumberToggle.getValue();
    }

    public boolean buttonState(GamepadButton button) {
        boolean value = false;

        switch (button) {
            case START:
                value = gamepad.start;
                break;
            case BACK:
                value = gamepad.back;
                break;
            case A_BUTTON:
                value = gamepad.a;
                break;
            case B_BUTTON:
                value = gamepad.b;
                break;
            case X_BUTTON:
                value = gamepad.x;
                break;
            case Y_BUTTON:
                value = gamepad.y;
                break;
            case LEFT_BUMPER:
                value = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                value = gamepad.right_bumper;
        }
        return value;
    }

}
