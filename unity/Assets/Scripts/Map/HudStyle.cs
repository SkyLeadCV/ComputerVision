using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The HUD's palette and drawing, shared by the map's panel and the simulator's onboard view so both read
    /// as one screen.</summary>
    public static class HudStyle
    {
        public static readonly Color Cyan = new(0.35f, 0.95f, 0.9f);
        public static readonly Color Dim = new(0.55f, 0.68f, 0.72f);
        public static readonly Color Panel = new(0.02f, 0.05f, 0.07f, 0.82f);
        public static readonly Color Amber = new(1f, 0.69f, 0.13f);
        public static readonly Color Red = new(1f, 0.25f, 0.25f);
        public static readonly Color Green = new(0.3f, 0.95f, 0.45f);

        // Laid out for 1080 p and scaled to the screen, so the HUD keeps its proportions at any resolution.
        public const float ReferenceHeightPx = 1080f;

        public static void Fill(Rect rect, Color color)
        {
            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }
    }
}
