using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The map's one typeface: a monospaced system font, so numbers keep their columns as they change.</summary>
    public static class MapFont
    {
        // Consolas ships with Windows, which the GPU PC runs; the others are fallbacks found on most systems.
        static readonly string[] Names = { "Consolas", "Cascadia Mono", "Courier New" };
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
