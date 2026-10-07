package org.firstinspires.ftc.teamcode.framework;

/** 子系統共同遵守的四個生命週期方法。 */
public abstract class OpBehavior {
    protected OpObject m_object;

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();
}
