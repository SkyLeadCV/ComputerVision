using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The map's light palette and HUD drawing, shared by the map's panels and the simulator's camera view so
    /// both read as one screen.</summary>
    public static class HudStyle
    {
        // Light and flat, like a printed plan: the background is what nobody has seen yet, white is floor the drone has
        // proven free, and a few saturated colours carry the things that matter.
        public static readonly Color Background = new(0.91f, 0.92f, 0.93f);
        public static readonly Color FreeFloor = Color.white;
        // The map's own floor before anything is proven, a step darker than the page, so unexplored ground inside the
        // map never reads as a hole showing the background.
        public static readonly Color Unexplored = new(0.80f, 0.82f, 0.85f);
        public static readonly Color Obstacle = new(0.20f, 0.22f, 0.25f);
        public static readonly Color DroneBlue = new(0.12f, 0.44f, 0.85f);
        public static readonly Color SeenNow = new(0.35f, 0.62f, 0.95f);
        // Warm, so the camera's blind zone never passes for the grey of floor nobody has seen.
        public static readonly Color BlindZone = new(0.96f, 0.60f, 0.22f);
        public static readonly Color VictimRed = new(0.88f, 0.19f, 0.19f);

        public static readonly Color Ink = new(0.13f, 0.15f, 0.17f);
        public static readonly Color Muted = new(0.32f, 0.35f, 0.39f);
        public static readonly Color Card = new(1f, 1f, 1f, 0.94f);
        public static readonly Color Border = new(0.82f, 0.84f, 0.87f);
        public static readonly Color Green = new(0.18f, 0.62f, 0.32f);
        public static readonly Color Amber = new(0.90f, 0.55f, 0.05f);

        // Laid out for 1080 p and scaled to the screen, so the HUD keeps its proportions at any resolution.
        public const float ReferenceHeightPx = 1080f;

        /// <summary>
        /// A palette colour as it must go into a mesh or line. The project renders in linear space, where vertex colours
        /// count as linear and show lighter than the same colour drawn by the HUD; converted, the map matches its legend.
        /// </summary>
        public static Color OnMap(Color color) =>
            QualitySettings.activeColorSpace == ColorSpace.Linear ? new Color(color.linear.r, color.linear.g, color.linear.b, color.a) : color;

        public static void Fill(Rect rect, Color color)
        {
            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }

        /// <summary>A white card with a thin border, the frame every HUD panel sits in.</summary>
        public static void CardFrame(Rect rect)
        {
            Fill(new Rect(rect.x - 1f, rect.y - 1f, rect.width + 2f, rect.height + 2f), Border);
            Fill(rect, Card);
        }

        public static GUIStyle Text(int size, Color color, FontStyle style = FontStyle.Normal, TextAnchor anchor = TextAnchor.MiddleLeft)
        {
            var text = new GUIStyle { font = MapFont.Get(), fontSize = size, fontStyle = style, alignment = anchor, richText = true };
            text.normal.textColor = color;
            return text;
        }
    }
}
