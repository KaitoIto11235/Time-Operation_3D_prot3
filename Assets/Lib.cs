using System.Runtime.InteropServices;

namespace NativePluginExample
{

    public static class Lib
    {
        [DllImport("NativePluginExample")]
        public static extern int DoSomething(int x, int y);

        [DllImport("NativePluginExample")]
        public static extern int ParticleFilter(double[] userTarget);

        // public static extern int ParticleFilter(float x, float y, float z);

        [DllImport("NativePluginExample")]
        public static extern int ResetTracker();
    }
}

