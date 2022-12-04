package org.firstinspires.ftc.robotcore.internal.network;

import com.qualcomm.robotcore.util.Device;

public class ApChannelManagerFactory
{
    protected static ApChannelManager apChannelManager = null;

    public static synchronized ApChannelManager getInstance()
    {
        if (apChannelManager == null) {
            if (Device.isRevControlHub()) {
                apChannelManager = new ControlHubApChannelManager();
            } else {
                apChannelManager = new WifiDirectChannelManager();
            }
            // TODO(Noah): On the Driver Station, build a RemoteChannelChanger
        }
        return apChannelManager;
    }

    //do not instantiate
    private ApChannelManagerFactory() {}
}
