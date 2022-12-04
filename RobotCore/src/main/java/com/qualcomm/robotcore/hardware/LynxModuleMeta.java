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
package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import androidx.annotation.Nullable;

/**
 * {@link LynxModuleMeta} has simple lynx module meta information for transmission from RC to DS
 */
@SuppressWarnings("WeakerAccess")
public class LynxModuleMeta
    {
    protected int moduleAddress;
    protected boolean isParent;
    @Nullable protected volatile ImuType imuType;

    public LynxModuleMeta(int moduleAddress, boolean isParent)
        {
        this.moduleAddress = moduleAddress;
        this.isParent = isParent;
        this.imuType = ImuType.UNKNOWN;
        }

    public LynxModuleMeta(LynxModuleMeta him)
        {
        this.moduleAddress = him.getModuleAddress();
        this.isParent = him.isParent();
        this.imuType = him.imuType;
        }

    public int getModuleAddress()
        {
        return moduleAddress;
        }

    public boolean isParent()
        {
        return isParent;
        }

    @Nullable public ImuType imuType()
        {
        return imuType;
        }

    public void setImuType(ImuType imuType)
        {
        this.imuType = imuType;
        }

    @Override public String toString()
        {
        return Misc.formatForUser("LynxModuleMeta(#%d,%b,ImuType.%s)", moduleAddress, isParent, imuType);
        }

    public enum ImuType { UNKNOWN, NONE, BNO055, BHI260 }
    }
