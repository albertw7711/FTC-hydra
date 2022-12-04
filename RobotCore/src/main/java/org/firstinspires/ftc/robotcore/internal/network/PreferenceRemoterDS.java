/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
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
package org.firstinspires.ftc.robotcore.internal.network;

import android.annotation.SuppressLint;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.SystemClock;

import com.qualcomm.robotcore.R;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;

/**
 * {@link PreferenceRemoterDS} is the driver station side of remoting of robot controller
 * preference settings.
 */
@SuppressWarnings("WeakerAccess")
public class PreferenceRemoterDS extends PreferenceRemoter
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String TAG = NetworkDiscoveryManager.TAG + "_prefremds";
    public String getTag() { return TAG; }

    @SuppressLint("StaticFieldLeak") protected static PreferenceRemoterDS theInstance = null;
    public synchronized static PreferenceRemoterDS getInstance()
        {
        if (null == theInstance) theInstance = new PreferenceRemoterDS();
        return theInstance;
        }

    protected PreferencesHelper.StringMap mapGroupOwnerToDeviceName;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public PreferenceRemoterDS()
        {
        // It's not critical for this code to work perfectly. If it gets confused because the user
        // changed the system time, the worst that can happen is for the wrong RC name to get displayed
        long maybeIncorrectSystemBootTime = System.currentTimeMillis() - SystemClock.elapsedRealtime();
        boolean rebootedSinceLastP2pRenameMapUpdate = maybeIncorrectSystemBootTime > getTimestampWhenRenameMapWasLastSaved();
        if (rebootedSinceLastP2pRenameMapUpdate) {
            RobotLog.vv(TAG, "The device has been booted since the last time the Driver Station was running");
            clearRenameMap();
        } else {
            loadRenameMap();
        }

        // Remove preferences that indicate state with temporary knowledge
        preferencesHelper.remove(context.getString(R.string.pref_wifip2p_groupowner_lastconnectedto));
        preferencesHelper.remove(context.getString(R.string.pref_wifip2p_channel));
        preferencesHelper.remove(context.getString(R.string.pref_has_independent_phone_battery_rc));
        }

    @Override
    protected SharedPreferences.OnSharedPreferenceChangeListener makeSharedPrefListener()
        {
        return new SharedPreferencesListenerDS();
        }

    //----------------------------------------------------------------------------------------------
    // Managing WifiP2p's refusal to track changes in robot controller name
    //----------------------------------------------------------------------------------------------

    public void onWifiToggled(boolean enabled)
        {
        RobotLog.vv(TAG, "onWifiToggled(%s)", enabled);
        if (!enabled)
            {
            clearRenameMap();
            }
        }

    protected void clearRenameMap()
        {
        RobotLog.vv(TAG, "clearRenameMap()");
        mapGroupOwnerToDeviceName = new PreferencesHelper.StringMap();
        saveRenameMap();
        }

    protected void saveRenameMap()
        {
        preferencesHelper.writeStringMapPrefIfDifferent(context.getString(R.string.pref_wifip2p_groupowner_map), mapGroupOwnerToDeviceName);
        preferencesHelper.writeLongPrefIfDifferent(context.getString(R.string.pref_wifip2p_groupowner_map_timestamp), System.currentTimeMillis());
        }

    protected void loadRenameMap()
        {
        mapGroupOwnerToDeviceName = preferencesHelper.readStringMap(context.getString(R.string.pref_wifip2p_groupowner_map), new PreferencesHelper.StringMap());
        }

    /**
     * @return The timestamp (in the format used by {@link System#currentTimeMillis()}) when the rename map was last saved
     */
    protected long getTimestampWhenRenameMapWasLastSaved()
        {
        return preferencesHelper.readLong(context.getString(R.string.pref_wifip2p_groupowner_map_timestamp), 0L);
        }

    public String getDeviceNameForWifiP2pGroupOwner(String groupOwnerName)
        {
        String result = mapGroupOwnerToDeviceName.get(groupOwnerName);
        return result != null ? result : groupOwnerName;
        }

    //----------------------------------------------------------------------------------------------
    // Remoting
    //----------------------------------------------------------------------------------------------

    @Override public CallbackResult handleCommandRobotControllerPreference(String extra)
        {
        RobotControllerPreference rcPrefAndValue = RobotControllerPreference.deserialize(extra);

        RobotLog.vv(getTag(), "handleRobotControllerPreference() pref=%s", rcPrefAndValue.getPrefName());

        // Some of the driver station's local settings will track corresponding settings
        // on the robot controller: the RC is the 'master', the DS the 'slave'
        if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_sound_on_off)))
            {
            // We're being told the sound setting of the robot controller
            if (preferencesHelper.readBoolean(context.getString(R.string.pref_has_speaker_rc),true))
                {
                preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_sound_on_off_rc), rcPrefAndValue.getValue());
                }
            else
                {
                // Can't have sound on if they have no speaker
                preferencesHelper.writeBooleanPrefIfDifferent(context.getString(R.string.pref_sound_on_off_rc), false);
                }
            }

        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_wifip2p_channel)))
            {
            // display channel for now.
            int prefChannel = (int)rcPrefAndValue.getValue();
            RobotLog.vv(TAG, "pref_wifip2p_channel: prefChannel = %d", prefChannel);

            // write this value so we can refer to it later on.
            preferencesHelper.writeIntPrefIfDifferent(context.getString(R.string.pref_wifip2p_channel), prefChannel);
            }

        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_has_speaker)))
            {
            // Remember whether the robot controller has a speaker
            preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_has_speaker_rc), rcPrefAndValue.getValue());

            // If they have no speaker, then their sound is off
            if (!preferencesHelper.readBoolean(context.getString(R.string.pref_has_speaker_rc),true))
                {
                preferencesHelper.writeBooleanPrefIfDifferent(context.getString(R.string.pref_sound_on_off_rc), false);
                }
            }


        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_app_theme)))
            {
            // Remember the theme of the robot controller as they told us it is
            preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_app_theme_rc), rcPrefAndValue.getValue());
            }

        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_device_name)))
            {
            // We're being told that the name of the robot controller has changed.

            // Remember the name of the robot controller as they told us it is
            preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_device_name_rc), rcPrefAndValue.getValue());

            // The robot controller *told*us* who he is. So that's the best name we can display for them
            preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_device_name_rc_display), rcPrefAndValue.getValue());

            // Finally, remember the map from the group owner we connected to to what they're called now.
            // You wouldn't think we'd have to do this, but it turns out that when the RC (the WifiP2p
            // group owner) changes its name, the group owner as reported by the system here on the
            // DS doesn't actually change until we reboot (or, perhaps, until we toggle and retoggle
            // our Wi-Fi). It'll all be fine when we reboot, but until then, we've got to keep the
            // names straight.
            String groupOwner = preferencesHelper.readString(context.getString(R.string.pref_wifip2p_groupowner_connectedto), "");
            if (groupOwner.isEmpty())
                groupOwner = preferencesHelper.readString(context.getString(R.string.pref_wifip2p_groupowner_lastconnectedto), "");

            if (!groupOwner.isEmpty())
                {
                String now = (String)rcPrefAndValue.getValue();
                mapGroupOwnerToDeviceName.put(groupOwner, now);
                saveRenameMap();
                }
            else
                {
                RobotLog.ee(TAG, "odd: we got a name change from an RC we're not actually connected to");
                }
            }

        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_wifip2p_remote_channel_change_works)))
            {
            // Just remember that here locally
            preferencesHelper.writePrefIfDifferent(rcPrefAndValue.getPrefName(), rcPrefAndValue.getValue());
            }

        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_has_independent_phone_battery)))
            {
            // Remember that setting of the robot controller
            preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_has_independent_phone_battery_rc), rcPrefAndValue.getValue());
            }
        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_warn_about_obsolete_software)))
            {
            preferencesHelper.writePrefIfDifferent(rcPrefAndValue.getPrefName(), rcPrefAndValue.getValue());
            }
        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_warn_about_mismatched_app_versions)))
            {
            preferencesHelper.writePrefIfDifferent(rcPrefAndValue.getPrefName(), rcPrefAndValue.getValue());
            }
        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_warn_about_2_4_ghz_band)))
            {
            preferencesHelper.writePrefIfDifferent(rcPrefAndValue.getPrefName(), rcPrefAndValue.getValue());
            }
        else if (rcPrefAndValue.getPrefName().equals(context.getString(R.string.pref_warn_about_incorrect_clocks)))
            {
            preferencesHelper.writePrefIfDifferent(rcPrefAndValue.getPrefName(), rcPrefAndValue.getValue());
            }
        return CallbackResult.HANDLED;
        }

    //----------------------------------------------------------------------------------------------
    // Preferences
    //----------------------------------------------------------------------------------------------

    protected class SharedPreferencesListenerDS implements SharedPreferences.OnSharedPreferenceChangeListener
        {
        @Override public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String dsPrefName)
            {
            RobotLog.vv(TAG, "onSharedPreferenceChanged(name=%s, value=%s)", dsPrefName, preferencesHelper.readPref(dsPrefName));

            String rcPrefName = null;
            if (dsPrefName.equals(context.getString(R.string.pref_sound_on_off_rc)))
                {
                rcPrefName = context.getString(R.string.pref_sound_on_off);
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_device_name_rc)))
                {
                rcPrefName = context.getString(R.string.pref_device_name);
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_app_theme_rc)))
                {
                rcPrefName = context.getString(R.string.pref_app_theme);
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_wifip2p_channel)))
                {
                rcPrefName = context.getString(R.string.pref_wifip2p_channel);
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_warn_about_obsolete_software)))
                {
                rcPrefName = dsPrefName;
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_warn_about_mismatched_app_versions)))
                {
                rcPrefName = dsPrefName;
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_warn_about_2_4_ghz_band)))
                {
                rcPrefName = dsPrefName;
                }
            else if (dsPrefName.equals(context.getString(R.string.pref_warn_about_incorrect_clocks)))
                {
                rcPrefName = dsPrefName;
                }
            if (rcPrefName != null)
                {
                Object value = preferencesHelper.readPref(dsPrefName);
                if (value != null)
                    {
                    sendPreference(new RobotControllerPreference(rcPrefName, value));
                    }
                }

            // Remember who we *were* connected to in case we disconnect
            if (dsPrefName.equals(context.getString(R.string.pref_wifip2p_groupowner_connectedto)))
                {
                String rcGroupOwnerName = preferencesHelper.readString(dsPrefName, "");
                if (!rcGroupOwnerName.isEmpty())
                    {
                    preferencesHelper.writePrefIfDifferent(context.getString(R.string.pref_wifip2p_groupowner_lastconnectedto), rcGroupOwnerName);
                    }
                else
                    {
                    RobotLog.vv(TAG, "%s has been removed", dsPrefName);
                    }
                }
            }
        }

    /**
     * Send some "preferences" that tell the RC information about ourselves
     */
    public void sendInformationalPrefsToRc()
        {
        int dsVersionCode = 0;
        try
            {
            //noinspection deprecation
            dsVersionCode = context.getPackageManager().getPackageInfo(context.getPackageName(), 0).versionCode;
            }
        catch (PackageManager.NameNotFoundException e) { e.printStackTrace(); } // shouldn't happen

        sendPreference(new RobotControllerPreference(context.getString(R.string.pref_ds_supports_5_ghz), WifiUtil.is5GHzAvailable()));
        if (Device.isRevDriverHub())
            {
            sendPreference(new RobotControllerPreference(context.getString(R.string.pref_dh_os_version_code), LynxConstants.getDriverHubOsVersionCode()));
            }
        }
    }
