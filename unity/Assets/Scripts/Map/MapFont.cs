using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The map's one typeface: a plain sans-serif from the system, so the HUD reads cleanly on any PC.</summary>
    public static class MapFont
    {
        // Segoe UI ships with Windows; the simulator also runs on a Linux PC, which has DejaVu or Liberation Sans.
        static readonly string[] Names = { "Segoe UI", "Helvetica Neue", "Arial", "DejaVu Sans", "Liberation Sans" };
        const int BaseSizePx = 32;

        static Font font;

        public static Font Get()
        {
            if (font == null)
            {
                font = Font.CreateDynamicFontFromOSFont(Names, BaseSizePx);
                font.hideFlags = HideFlags.DontSave;
            }
            return font;
        }
    }
}
