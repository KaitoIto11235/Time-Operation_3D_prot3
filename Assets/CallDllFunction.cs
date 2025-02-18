using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NativePluginExample
{
    public class CallDllFunction : MonoBehaviour
    {
        float time = 0f;
        [SerializeField] GameObject modelObj;
        [SerializeField] GameObject userObj;
        [SerializeField] GameObject userTargetObj;
        double[] model = new double[3];
        double[] userObserve = new double[3];
        double[] userTarget = new double[3];
        void Start()
        {
            Debug.Log(Lib.DoSomething(3, 4));
        }

        void Update()
        {
            // 前のフレームから何秒かかったか
            time += Time.deltaTime;
            Debug.Log(time);
            Debug.Log(Time.deltaTime);

            Lib.ParticleFilter(model, userObserve, userTarget);

            modelObj.transform.position = new Vector3((float)model[0], (float)model[1], (float)model[2]);
            userObj.transform.position = new Vector3((float)userObserve[0], (float)userObserve[1], (float)userObserve[2]);
            userTargetObj.transform.position = new Vector3((float)userTarget[0], (float)userTarget[1], (float)userTarget[2]);

            if (Input.GetKeyDown(KeyCode.Return))
            {
                Debug.Log("Enter!");
                Lib.ResetCount();
            }
        }

        void OnApplicationQuit()
        {
            Debug.Log("Finish!");
            Lib.FinishTracker();
        }

    }
}

