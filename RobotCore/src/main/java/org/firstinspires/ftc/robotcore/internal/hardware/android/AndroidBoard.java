/*
Copyright (c) 2018 Noah Andrews

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Noah Andrews nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.internal.hardware.android;

import com.qualcomm.robotcore.R;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Intents;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public abstract class AndroidBoard {
    private static final String TAG = "AndroidBoard";

    private static AndroidBoard androidBoard;

    // GPIO pin names
    protected static final String ANDROID_BOARD_IS_PRESENT_PIN_NAME = "AndroidBoardIsPresentPin";
    protected static final String LYNX_MODULE_RESET_PIN_NAME = "LynxModuleResetPin";
    protected static final String PROGRAMMING_PIN_NAME = "ProgrammingPin";
    protected static final String USER_BUTTON_PIN_NAME = "UserButtonPin";
    protected static final String BHI_260_QUATERNION_REGISTER_FREEZE_PIN_NAME = "BHI260QuatFreezePin";

    /**
     * Get the instance of AndroidBoard that is appropriate for our hardware
     *
     * Returns a FakeAndroidBoard if the Control Hub type is unknown.
     */
    public static AndroidBoard getInstance() {
        if (androidBoard == null) {
            if (isRevControlHubv1()) {
                androidBoard = new Rev3328();
            } else if (isDragonboard()) {
                androidBoard = new Dragonboard();
            } else {
                androidBoard = new FakeAndroidBoard();
            }
        }
        return androidBoard;
    }

    /**
     * Show an error message if we're running on an unrecognized Control Hub
     */
    public static void showErrorIfUnknownControlHub() {
        if (Device.isRevControlHub()) {
            if (androidBoard == null) {
                getInstance(); // Initializes androidBoard
            }

            if (androidBoard instanceof FakeAndroidBoard) {
                RobotLog.setGlobalErrorMsg(AppUtil.getDefContext().getString(R.string.unknown_control_hub_error));
            }
        }
    }

    /**
     * Returns the type of Android board we're running on as a String
     *
     * Intended only for logging or display. Device-specific functionality belongs in the appropriate
     * subclass of AndroidBoard.
     */
    public abstract String getDeviceType();

     // GPIO pins

    /**
     * Returns a {@link DigitalChannel} that controls the AndroidBoardIsPresent pin
     * (also known as the DragonboardIsPresent pin) (96boards pin 8)
     */
    public abstract DigitalChannel getAndroidBoardIsPresentPin();

    /**
     * Returns a {@link DigitalChannel} that controls the Lynx programming pin (96boards pin 23)
     */
    public abstract DigitalChannel getProgrammingPin();

    /**
     * Returns a {@link DigitalChannel} that controls the Lynx reset pin (96boards pin 25)
     */
    public abstract DigitalChannel getLynxModuleResetPin();

    /**
     * Returns a {@link DigitalChannel} that gives the value of the user-facing button on the
     * Control Hub (96boards pin 10)
     */
    public abstract DigitalChannel getUserButtonPin();

    /**
     * Returns a {@link DigitalChannel} that controls the BHI260 quaternion register freeze pin
     * (96boards pin 30)
     */
    public abstract DigitalChannel getBhi260QuatRegFreezePin();

    /**
     * Returns a {@link File} that points to the location of the "file" that we read and write from
     * to communicate with the Lynx board over UART
     */
    public abstract File getUartLocation();

    /**
     * Returns whether or not this Android board supports broadcasting a 5GHz Wi-Fi access point
     */
    public abstract boolean supports5GhzAp();

    /**
     * Returns whether or not this Android board supports auto-selecting a 5GHz Wi-Fi channel
     */
    public abstract boolean supports5GhzAutoSelection();

    /**
     * Returns whether or not the AP service supports setting the network properties in bulk
     */
    public abstract boolean supportsBulkNetworkSettings();

    /**
     * Returns whether or not the AP service supports {@link Intents#ACTION_FTC_AP_GET_CURRENT_CHANNEL_INFO}
     */
    public abstract boolean supportsGetChannelInfoIntent();

    /**
     * Returns the current data rate of our Wi-Fi access point beacons
     */
    public abstract WifiDataRate getWifiApBeaconRate();

    /**
     * Set the data rate of our Wi-Fi access point beacons
     *
     * Should fail silently if this is not possible
     */
    public abstract void setWifiApBeaconRate(WifiDataRate beaconRate);

    /**
     * Returns whether or not the board's OS has ControlHubUpdater baked into its OS
     */
    public abstract boolean hasControlHubUpdater();

    /**
     * Returns whether or not the Control Hub OS has a watchdog for the RC app that looks for
     * {@link Intents#ACTION_FTC_NOTIFY_RC_ALIVE} broadcasts
     */
    public abstract boolean hasRcAppWatchdog();

    /**
     * Logs some basic info about the active Android board.
     */
    public void logAndroidBoardInfo() {
        if (Device.isRevControlHub()) {
            RobotLog.vv(TAG, "REV Control Hub contains " + getDeviceType());
            RobotLog.vv(TAG, "Communicating with embedded REV hub via " + getUartLocation().getAbsolutePath());
        }
    }

    /**
     * Answers whether this is a Dragonboard
     */
    private static boolean isDragonboard() {
        return LynxConstants.getControlHubVersion() == 0;
    }

    /**
     * Answers whether this is a Control Hub V1 (based on the RK3328 processor)
     */
    private static boolean isRevControlHubv1() {
        return LynxConstants.getControlHubVersion() == 1;
    }

    public enum WifiDataRate {
        UNKNOWN,
        CCK_1Mb,
        CCK_2Mb,
        CCK_5Mb,
        CCK_11Mb,
        OFDM_6Mb,
        OFDM_9Mb,
        OFDM_12Mb,
        OFDM_18Mb,
        OFDM_24Mb,
        OFDM_36Mb,
        OFDM_48Mb,
        OFDM_54Mb;
    }
}
