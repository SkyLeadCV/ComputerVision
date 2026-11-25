using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.InputSystem;
using static SkyLead.Map.HudStyle;

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

        // The key of MapView's switch between views, named in the legend; the map's camera reads the key itself.
        const string ViewKeyName = "V";
        const float MarginPx = 24f;
        const float PanelWidthPx = 350f;
        const float PaddingPx = 16f;
        const float HeaderHeightPx = 60f;
        const float RowHeightPx = 32f;
        const float LabelWidthPx = 110f;
        const float LegendWidthPx = 300f;
        const float LegendRowPx = 30f;
        const float SwatchPx = 16f;
        const float KeysHeightPx = 30f;

        string mode;
        float personDistanceCm = -1f;
        string endReason;
        GUIStyle title, caption, label, value, tagStyle;

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
            CreateStyles();
            DrawStatus();
            DrawLegend();
        }

        void DrawStatus()
        {
            List<(string label, string value)> rows = Rows;
            var card = new Rect(MarginPx, MarginPx, PanelWidthPx, PaddingPx * 2f + HeaderHeightPx + rows.Count * RowHeightPx);
            CardFrame(card);
            float x = card.x + PaddingPx, y = card.y + PaddingPx;
            GUI.Label(new Rect(x, y, PanelWidthPx, 24f), "SkyLead", title);
            GUI.Label(new Rect(x, y + 24f, PanelWidthPx, 20f), "Search and rescue map", caption);
            y += HeaderHeightPx;

            foreach ((string name, string text) in rows)
            {
                GUI.Label(new Rect(x, y, LabelWidthPx, RowHeightPx), Shown(name), label);
                var valueRect = new Rect(x + LabelWidthPx, y, PanelWidthPx - LabelWidthPx - 2f * PaddingPx, RowHeightPx);
                switch (name)
                {
                    case "LINK":
                        Dot(new Vector2(valueRect.x + 5f, valueRect.center.y), 5f, LinkColor());
                        GUI.Label(new Rect(valueRect.x + 16f, valueRect.y, valueRect.width - 16f, valueRect.height), Title(Link.ToString()), value);
                        break;
                    case "MODE":
                        string shownMode = Title(text);
                        float tagWidth = tagStyle.CalcSize(new GUIContent(shownMode)).x + 18f;
                        var tag = new Rect(valueRect.x, valueRect.y + 4f, tagWidth, RowHeightPx - 8f);
                        Fill(tag, ModeColor());
                        GUI.Label(tag, shownMode, tagStyle);
                        break;
                    default:
                        Color previous = GUI.contentColor;
                        if (name == "VICTIMS" && Victims > 0)
                            GUI.contentColor = VictimRed;
                        else if (name == "END")
                            GUI.contentColor = Amber;
                        GUI.Label(valueRect, name == "END" || name == "RESCUER" ? Title(text) : text, value);
                        GUI.contentColor = previous;
                        break;
                }
                y += RowHeightPx;
            }
        }

        /// <summary>What every colour on the map means, always on screen so the view explains itself.</summary>
        void DrawLegend()
        {
            (string text, Color color, Swatch swatch)[] entries =
            {
                ("Floor proven free", FreeFloor, Swatch.Square),
                ("Not explored yet", Unexplored, Swatch.Square),
                ("Obstacle seen", Obstacle, Swatch.Dot),
                ("Floor in the last photo", new Color(SeenNow.r, SeenNow.g, SeenNow.b, 0.45f), Swatch.Square),
                ("Camera blind zone", new Color(BlindZone.r, BlindZone.g, BlindZone.b, 0.45f), Swatch.Square),
                ("Drone and its path", DroneBlue, Swatch.Line),
                ("Victim", VictimRed, Swatch.Dot),
            };
            float height = PaddingPx * 2f + entries.Length * LegendRowPx + KeysHeightPx;
            var card = new Rect(MarginPx, ReferenceHeightPx - MarginPx - height, LegendWidthPx, height);
            CardFrame(card);
            float x = card.x + PaddingPx, y = card.y + PaddingPx;
            foreach ((string text, Color color, Swatch swatch) in entries)
            {
                var box = new Rect(x, y + (LegendRowPx - SwatchPx) / 2f, SwatchPx, SwatchPx);
                switch (swatch)
                {
                    case Swatch.Square:
                        Fill(new Rect(box.x - 1f, box.y - 1f, box.width + 2f, box.height + 2f), Border);
                        Fill(box, Background);
                        Fill(box, color);
                        break;
                    case Swatch.Dot:
                        Dot(box.center, SwatchPx / 2f, color);
                        break;
                    case Swatch.Line:
                        Fill(new Rect(box.x, box.center.y - 1.5f, box.width, 3f), color);
                        break;
                }
                GUI.Label(new Rect(x + SwatchPx + 12f, y, LegendWidthPx, LegendRowPx), text, label);
                y += LegendRowPx;
            }
            GUI.Label(new Rect(x, y + 6f, LegendWidthPx, KeysHeightPx - 6f), $"<b>{ViewKeyName}</b>  3D view     <b>{toggleKey}</b>  hide text", caption);
        }

        enum Swatch { Square, Dot, Line }

        static void Dot(Vector2 centre, float radius, Color color)
        {
            // Two crossed bars read as a dot at this size, and IMGUI draws no circles.
            Fill(new Rect(centre.x - radius, centre.y - radius * 0.6f, radius * 2f, radius * 1.2f), color);
            Fill(new Rect(centre.x - radius * 0.6f, centre.y - radius, radius * 1.2f, radius * 2f), color);
        }

        static string Shown(string name) => name switch
        {
            "T+" => "Time",
            "END" => "Ended",
            _ => Title(name),
        };

        static string Title(string text)
        {
            string words = text.Replace('_', ' ').ToLowerInvariant();
            return words.Length == 0 ? words : char.ToUpperInvariant(words[0]) + words.Substring(1);
        }

        Color LinkColor() => Link switch
        {
            MapLink.Live => Green,
            MapLink.Replay => Amber,
            MapLink.Lost => VictimRed,
            _ => Muted,
        };

        Color ModeColor() => mode switch
        {
            "exploring" => DroneBlue,
            "waiting" => Amber,
            "guiding" => Green,
            "arrived" => VictimRed,
            _ => Muted,
        };

        void CreateStyles()
        {
            if (title != null)
                return;
            title = Text(22, Ink, FontStyle.Bold);
            caption = Text(15, Muted);
            label = Text(17, Muted);
            value = Text(17, Ink, FontStyle.Bold);
            tagStyle = Text(14, Color.white, FontStyle.Bold, TextAnchor.MiddleCenter);
        }
    }
}
