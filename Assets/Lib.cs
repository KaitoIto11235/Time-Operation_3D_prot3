using System.Runtime.InteropServices;

namespace NativePluginExample
{

    public static class Lib
    {
        [DllImport("NativePluginExample")]
        public static extern int DoSomething(int x, int y);
    }
}

