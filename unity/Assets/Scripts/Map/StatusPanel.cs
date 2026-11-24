using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.InputSystem;

namespace SkyLead.Map
{
    public enum MapLink
    {
        Waiting,
        Live,
        Replay,
        Lost,
    }

    /// <summary>The mission HUD: where the messages come from, the mission clock and phase, victims and rescuer.</summary>
    public sealed class StatusPanel : MonoBehaviour
    {
        [Tooltip("Shows or hides every text on the map, so the view can be recorded for the presentation without it.")]
        [SerializeField] Key toggleKey = Key.H;

        // Laid out for 1080 p and scaled to the screen, so the HUD keeps its proportions at any resolution.
        const float ReferenceHeightPx = 1080f;
        const float MarginPx = 28f;
        const float PanelWidthPx = 420f;
        const float PaddingPx = 18f;
        const float RowHeightPx = 30f;
        const float LabelWidthPx = 120f;
        const float BracketPx = 36f;

        static readonly Color Cyan = new(0.35f, 0.95f, 0.9f);
        static readonly Color Dim = new(0.55f, 0.68f, 0.72f);
        static readonly Color Panel = new(0.02f, 0.05f, 0.07f, 0.82f);
        static readonly Color Amber = new(1f, 0.69f, 0.13f);
        static readonly Color Red = new(1f, 0.25f, 0.25f);
        static readonly Color Green = new(0.3f, 0.95f, 0.45f);

        string mode;
        float personDistanceCm = -1f;
        string endReason;
        GUIStyle title, label, value, tagStyle, footer;

        public MapLink Link { get; set; }

        /// <summary>The address listened on, or the recording being replayed.</summary>
        public string LinkDetail { get; set; } = "";

        /// <summary>Seconds since session_start, from the latest message's t.</summary>
        public float MissionTimeS { get; set; }

        public bool TextVisible { get; private set; } = true;

        int Victims => GetComponent<VictimMarkers>().Count;

        /// <summary>The HUD's rows as label and value, also what the tests read.</summary>
        public List<(string label, string value)> Rows
        {
            get
            {
                var rows = new List<(string, string)>
                {
                    ("LINK", $"{Link.ToString().ToUpperInvariant()}  {LinkDetail}"),
                    ("MODE", mode == null ? "STANDBY" : mode.ToUpperInvariant()),
                    ("T+", Clock(MissionTimeS)),
                    ("VICTIMS", Victims.ToString(CultureInfo.InvariantCulture)),
                    // -1 means no rescuer in view (docs/protocol.md).
                    ("RESCUER", personDistanceCm >= 0f ? Metres(personDistanceCm) : "NO CONTACT"),
                };
                if (endReason != null)
                    // Shown as it arrives: the brain can still report back_at_start without having made it back.
                    rows.Add(("END", endReason.ToUpperInvariant()));
                return rows;
            }
        }

        public string Text => string.Join("\n", Rows.ConvertAll(row => $"{row.label} {row.value}"));

        public void ShowState(MissionState state)
        {
            mode = state.mode;
            personDistanceCm = state.person_distance_cm;
        }

        public void ShowEnd(SessionEnd end) => endReason = end.reason;

        public void Clear()
        {
            mode = null;
            personDistanceCm = -1f;
            endReason = null;
            MissionTimeS = 0f;
        }

        // Invariant, so a Spanish Windows still writes "1.8 m" in the English HUD.
        static string Metres(float cm) => (cm / 100f).ToString("0.0", CultureInfo.InvariantCulture) + " m";

        static string Clock(float seconds)
        {
            int whole = Mathf.Max(0, Mathf.FloorToInt(seconds));
            return $"{whole / 60:00}:{whole % 60:00}";
        }

        void Update()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null || !keyboard[toggleKey].wasPressedThisFrame)
                return;
            TextVisible = !TextVisible;
            GetComponent<VictimMarkers>().ShowLabels = TextVisible;
        }

        void OnGUI()
        {
            if (!TextVisible)
                return;
            float scale = Screen.height / ReferenceHeightPx;
            GUI.matrix = Matrix4x4.Scale(new Vector3(scale, scale, 1f));
            float width = Screen.width / scale, height = ReferenceHeightPx;
            CreateStyles();

            DrawCornerBrackets(width, height);

            List<(string label, string value)> rows = Rows;
            float panelHeight = PaddingPx * 2f + 56f + rows.Count * RowHeightPx;
            var panel = new Rect(MarginPx, MarginPx, PanelWidthPx, panelHeight);
            Fill(panel, Panel);
            Fill(new Rect(panel.x, panel.y, 3f, panel.height), Cyan);
            Fill(new Rect(panel.x, panel.y, panel.width, 1f), new Color(Cyan.r, Cyan.g, Cyan.b, 0.5f));

            float x = panel.x + PaddingPx, y = panel.y + PaddingPx;
            GUI.Label(new Rect(x, y, PanelWidthPx, 26f), "SKYLEAD <color=#7FA3AA>//  SEARCH & RESCUE</color>", title);
            y += 34f;
            Fill(new Rect(x, y, PanelWidthPx - 2f * PaddingPx, 1f), new Color(Dim.r, Dim.g, Dim.b, 0.35f));
            y += 22f;

            foreach ((string name, string text) in rows)
            {
                GUI.Label(new Rect(x, y, LabelWidthPx, RowHeightPx), name, label);
                var valueRect = new Rect(x + LabelWidthPx, y, PanelWidthPx - LabelWidthPx - 2f * PaddingPx, RowHeightPx);
                switch (name)
                {
                    case "LINK":
                        Fill(new Rect(valueRect.x, valueRect.y + 9f, 10f, 10f), LinkColor());
                        GUI.Label(new Rect(valueRect.x + 18f, valueRect.y, valueRect.width - 18f, valueRect.height), text, value);
                        break;
                    case "MODE":
                        float tagWidth = tagStyle.CalcSize(new GUIContent(text)).x + 16f;
                        Fill(new Rect(valueRect.x, valueRect.y + 3f, tagWidth, RowHeightPx - 8f), ModeColor());
                        GUI.Label(new Rect(valueRect.x, valueRect.y + 3f, tagWidth, RowHeightPx - 8f), text, tagStyle);
                        break;
                    default:
                        Color previous = GUI.contentColor;
                        if (name == "VICTIMS" && Victims > 0)
                            GUI.contentColor = Red;
                        else if (name == "END")
                            GUI.contentColor = Amber;
                        GUI.Label(valueRect, text, value);
                        GUI.contentColor = previous;
                        break;
                }
                y += RowHeightPx;
            }

            GUI.Label(new Rect(MarginPx, height - MarginPx - 20f, 300f, 20f), $"[{toggleKey}]  HIDE HUD", footer);
        }

        Color LinkColor() => Link switch
        {
            MapLink.Live => Green,
            MapLink.Replay => Amber,
            MapLink.Lost => Red,
            _ => Dim,
        };

        Color ModeColor() => mode switch
        {
            "exploring" => new Color(0.2f, 0.6f, 1f),
            "waiting" => Amber,
            "guiding" => Green,
            "arrived" => new Color(1f, 0.85f, 0.3f),
            _ => new Color(Dim.r, Dim.g, Dim.b, 0.6f),
        };

        void DrawCornerBrackets(float width, float height)
        {
            var color = new Color(Cyan.r, Cyan.g, Cyan.b, 0.45f);
            const float inset = 12f, thickness = 2f;
            foreach ((float cx, float cy, float dx, float dy) in new[]
            {
                (inset, inset, 1f, 1f), (width - inset, inset, -1f, 1f),
                (inset, height - inset, 1f, -1f), (width - inset, height - inset, -1f, -1f),
            })
            {
                Fill(new Rect(dx > 0 ? cx : cx - BracketPx, dy > 0 ? cy : cy - thickness, BracketPx, thickness), color);
                Fill(new Rect(dx > 0 ? cx : cx - thickness, dy > 0 ? cy : cy - BracketPx, thickness, BracketPx), color);
            }
        }

        static void Fill(Rect rect, Color color)
        {
            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }

        void CreateStyles()
        {
            if (title != null)
                return;
            Font font = MapFont.Get();
            title = new GUIStyle { font = font, fontSize = 20, fontStyle = FontStyle.Bold, richText = true };
            title.normal.textColor = Cyan;
            label = new GUIStyle { font = font, fontSize = 15, alignment = TextAnchor.MiddleLeft };
            label.normal.textColor = Dim;
            value = new GUIStyle { font = font, fontSize = 18, alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
            value.normal.textColor = Color.white;
            tagStyle = new GUIStyle { font = font, fontSize = 15, alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold };
            tagStyle.normal.textColor = new Color(0.02f, 0.05f, 0.07f);
            footer = new GUIStyle { font = font, fontSize = 13 };
            footer.normal.textColor = Dim;
        }
    }
}
