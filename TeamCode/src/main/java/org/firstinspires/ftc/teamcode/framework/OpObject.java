package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

/** FTC 的小寫回呼與本專案大寫生命週期之間的橋樑。 */
public abstract class OpObject extends OpMode {
    private final List<OpBehavior> m_behaviors = new ArrayList<>();
    private boolean registering = false;

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();

    /** 只在模式的 Init() 中登記，確保子系統不會錯過初始化。 */
    protected final void RegisterBehavior(OpBehavior behavior) {
        if (!registering || behavior == null || m_behaviors.contains(behavior)) {
            throw new IllegalStateException("請在 Init() 中登記一個尚未登記的子系統");
        }
        behavior.m_object = this;
        m_behaviors.add(behavior);
    }

    @Override
    public final void init() {
        registering = true;
        try {
            Init();
        } finally {
            registering = false;
        }
        for (OpBehavior behavior : m_behaviors) behavior.Init();
    }

    @Override
    public final void start() {
        Start();
        for (OpBehavior behavior : m_behaviors) behavior.Start();
    }

    @Override
    public final void loop() {
        Loop();
        for (OpBehavior behavior : m_behaviors) behavior.Loop();
    }

    @Override
    public final void stop() {
        try {
            Stop();
        } finally {
            for (OpBehavior behavior : m_behaviors) {
                try {
                    behavior.Stop();
                } finally {
                    behavior.m_object = null;
                }
            }
        }
    }
}
