// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class LaunchpadButton extends Trigger {
    public LaunchpadButton(Launchpad launchpad, int row, int col)
    {
        super(()->launchpad.isPressed(row, col));
    }
}