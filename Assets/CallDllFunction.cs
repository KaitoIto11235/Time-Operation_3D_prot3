using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NativePluginExample
{
    public class CallDllFunction : MonoBehaviour
    {
        [SerializeField] GameObject user;
        [SerializeField] GameObject userTargetObj;
        double[] userObserve = new double[3];
        double[] userTarget = new double[3];
        void Start()
        {
            Debug.Log(Lib.DoSomething(3, 4));
        }

        void Update()
        {
            userObserve[0] = user.transform.position.x;
            userObserve[1] = user.transform.position.y; 
            userObserve[2] = user.transform.position.z;

            Lib.ParticleFilter(userObserve, userTarget);
            userTargetObj.transform.position = new Vector3((float)userTarget[0], (float)userTarget[1], (float)userTarget[2]);

            if (Input.GetKeyDown(KeyCode.Return))
            {
                Debug.Log("Enter!");
                Lib.ResetCount();
            }
        }

        void Dispose()
        {
            Debug.Log("Finish!");
            Lib.FinishTracker();
        }

    }
}

