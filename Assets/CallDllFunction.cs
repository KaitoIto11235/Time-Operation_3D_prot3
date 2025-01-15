using UnityEngine;

namespace NativePluginExample
{
    public class CallDllFunction : MonoBehaviour
    {
        double[] userTarget = new double[3];
        void Start()
        {
            Debug.Log(Lib.DoSomething(3, 4));
            Lib.ParticleFilter(userTarget);
            Debug.Log(userTarget[0]);
            Debug.Log(userTarget[1]);
            Debug.Log(userTarget[2]);
            Lib.ResetTracker();
        }

    }
}

