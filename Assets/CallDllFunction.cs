using UnityEngine;

namespace NativePluginExample
{
    public class CallDllFunction : MonoBehaviour
    {
        // Start is called before the first frame update
        void Start()
        {
            Debug.Log(Lib.DoSomething(3, 4));
        }

    }
}

