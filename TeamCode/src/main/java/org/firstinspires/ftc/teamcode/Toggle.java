package org.firstinspires.ftc.teamcode;

public class Toggle {
    private boolean state = false;

    private boolean buttonPreviouslyPressed = false;

    /**
     * Constructor for the Toggle.
     * @param initialState The state the toggle should start in (true for ON, false for OFF).
     */
    public Toggle(boolean initialState)
    {
        this.state = initialState;
    }
    /**
     * Updates the toggle's state based on the current state of the button.
     * This method must be called in every loop cycle for the toggle to work.
     *
     * @param currentState The current state of the button (e.g., gamepad1.a).
     */
    public void updateToggle(boolean currentState)
    {
        if(currentState && !buttonPreviouslyPressed)
        {
            state = !state;
        }
        buttonPreviouslyPressed = currentState;
    }

    public boolean getState()
    {
        return this.state;
    }
    public void setState(boolean state)
    {
        this.state = state;
    }



}
