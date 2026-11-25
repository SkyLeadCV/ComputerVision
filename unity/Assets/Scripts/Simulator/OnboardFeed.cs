using SkyLead.Map;
using UnityEngine;
using static SkyLead.Map.HudStyle;

namespace SkyLead.Simulator
{
    /// <summary>
    /// The drone's camera live in a corner of the map, titled so it is always clear what it shows, and marked each time
    /// the brain takes a photo: what the drone sees next to what the map makes of it.
    /// </summary>
    [RequireComponent(typeof(SimulatedDrone))]
    public sealed class OnboardFeed : MonoBehaviour
    {
        [SerializeField] TelloCamera telloCamera;

        // Half the photo's 960 x 720: enough for a corner of the screen, and the photos themselves are rendered apart.
        const int FeedWidthPx = 480;
        const int FeedHeightPx = 360;
        const float MarginPx = 28f;
        const float InsetWidthPx = 440f;
        const float TitleHeightPx = 26f;
        const float PhotoNoticeS = 0.8f;

        RenderTexture feed;
        StatusPanel hud;
        GUIStyle title, caption;

        SimulatedDrone Drone => GetComponent<SimulatedDrone>();

        void Start()
        {
            feed = new RenderTexture(FeedWidthPx, FeedHeightPx, 24) { antiAliasing = 4, name = "Onboard feed" };
            var view = telloCamera.GetComponent<Camera>();
            // Into a texture, never the screen, which belongs to the map's camera.
            view.targetTexture = feed;
            view.enabled = true;
        }

        void OnDestroy()
        {
            if (feed != null)
                feed.Release();
        }

        void OnGUI()
        {
            float scale = Screen.height / ReferenceHeightPx;
            GUI.matrix = Matrix4x4.Scale(new Vector3(scale, scale, 1f));
            float width = Screen.width / scale;
            float imageHeight = InsetWidthPx * FeedHeightPx / FeedWidthPx;
            var image = new Rect(width - MarginPx - InsetWidthPx, ReferenceHeightPx - MarginPx - imageHeight, InsetWidthPx, imageHeight);
            var bar = new Rect(image.x, image.y - TitleHeightPx, image.width, TitleHeightPx);

            Fill(new Rect(bar.x - 1f, bar.y - 1f, bar.width + 2f, bar.height + image.height + 2f), Panel);
            GUI.DrawTexture(image, feed, ScaleMode.ScaleToFit, false);

            // The image stays for recordings; only the text follows the map's H key.
            hud ??= FindAnyObjectByType<StatusPanel>();
            if (hud != null && !hud.TextVisible)
                return;
            CreateStyles();
            GUI.Label(new Rect(bar.x + 10f, bar.y, 200f, bar.height), "DRONE CAMERA", title);
            bool photo = Time.unscaledTime - telloCamera.LastPhotoTime < PhotoNoticeS;
            string state = $"{Mathf.RoundToInt(Drone.PositionCm.z)} cm · {Mathf.RoundToInt(Mathf.Repeat(Drone.YawDeg, 360f)):000}°";
            caption.alignment = TextAnchor.MiddleRight;
            GUI.Label(new Rect(bar.x, bar.y, bar.width - 10f, bar.height), photo ? "photo taken" : state, caption);
        }

        void CreateStyles()
        {
            if (title != null)
                return;
            title = new GUIStyle { font = MapFont.Get(), fontSize = 14, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleLeft };
            title.normal.textColor = Color.white;
            caption = new GUIStyle { font = MapFont.Get(), fontSize = 13 };
            caption.normal.textColor = Dim;
        }
    }
}
