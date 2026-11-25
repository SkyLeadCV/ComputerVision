using System.Collections.Generic;
using UnityEngine;
using static SkyLead.Map.HudStyle;

namespace SkyLead.Map
{
    /// <summary>A red marker for every victim: a disc on the floor with a slowly pulsing ring, a thin beam for the 3D
    /// view and its tag. A repeated id moves its marker (docs/protocol.md).</summary>
    public sealed class VictimMarkers : MonoBehaviour
    {
        [Tooltip("Unlit, vertex-coloured material for the markers.")]
        [SerializeField] Material overlayMaterial;

        // Taller than the tables (about 75 cm), so the beam shows from the 3D view over the furniture.
        const float BeamHeightM = 1.3f;
        const float BeamWidthM = 0.02f;
        const float RingWidthM = 0.015f;
        // About a person's shoulder width.
        const float DiscRadiusM = 0.25f;
        const float PulseMinRadiusM = 0.25f;
        const float PulseMaxRadiusM = 0.55f;
        const float PulsePeriodS = 2f;
        const float FloorLiftM = 0.012f;
        const float LabelGapM = 0.06f;
        const float LabelCharacterSizeM = 0.04f;
        const int LabelFontSize = 64;

        readonly Dictionary<int, Transform> markers = new();
        readonly List<LineRenderer> rings = new();
        readonly List<TextMesh> labels = new();
        bool showLabels = true;

        public int Count => markers.Count;

        public bool ShowLabels
        {
            set
            {
                showLabels = value;
                foreach (TextMesh label in labels)
                    label.gameObject.SetActive(value);
            }
        }

        public Vector3 PositionOf(int id) => markers[id].localPosition;

        public void Place(Victim victim)
        {
            if (!markers.TryGetValue(victim.id, out Transform marker))
            {
                marker = CreateMarker(victim.id);
                markers.Add(victim.id, marker);
            }
            marker.localPosition = VoxelGrid.ToUnity(victim.x, victim.y, 0f);
        }

        public void Clear()
        {
            foreach (Transform marker in markers.Values)
            {
                if (Application.isPlaying)
                    Destroy(marker.gameObject);
                else
                    DestroyImmediate(marker.gameObject);
            }
            markers.Clear();
            rings.Clear();
            labels.Clear();
        }

        void Update()
        {
            // A slow ring that grows and fades, so a victim catches the eye without flashing.
            float phase = Time.time / PulsePeriodS % 1f;
            float radius = Mathf.Lerp(PulseMinRadiusM, PulseMaxRadiusM, phase);
            var color = new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0.6f * (1f - phase));
            foreach (LineRenderer ring in rings)
            {
                ring.transform.localScale = Vector3.one * radius;
                ring.startColor = ring.endColor = OnMap(color);
            }
        }

        void LateUpdate()
        {
            // Tags face the camera so they read from any view.
            Camera view = Camera.main;
            if (view == null)
                return;
            foreach (TextMesh label in labels)
                label.transform.rotation = view.transform.rotation;
        }

        Transform CreateMarker(int id)
        {
            var marker = new GameObject($"Victim {id}") { hideFlags = HideFlags.DontSave }.transform;
            marker.SetParent(transform, false);
            var lift = new Vector3(0f, FloorLiftM, 0f);

            MeshRenderer disc = MapShapes.MeshObject(marker, "Disc", MapShapes.Disc(DiscRadiusM, new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0.9f),
                new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0.75f)), overlayMaterial);
            disc.transform.localPosition = lift;
            disc.sortingOrder = MapLayers.Victims;

            // A unit circle scaled by the pulse; the line's width does not scale with it.
            LineRenderer ring = MapShapes.Line(marker, "Pulse", overlayMaterial, RingWidthM, true);
            ring.transform.localPosition = lift;
            MapShapes.SetCircle(ring, 1f);
            ring.transform.localScale = Vector3.one * PulseMinRadiusM;
            ring.startColor = ring.endColor = OnMap(new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0f));
            ring.sortingOrder = MapLayers.Victims;
            rings.Add(ring);

            LineRenderer beam = MapShapes.Line(marker, "Beam", overlayMaterial, BeamWidthM, false);
            beam.positionCount = 2;
            beam.SetPosition(0, Vector3.zero);
            beam.SetPosition(1, new Vector3(0f, BeamHeightM, 0f));
            beam.startColor = OnMap(new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0.8f));
            beam.endColor = OnMap(new Color(VictimRed.r, VictimRed.g, VictimRed.b, 0f));
            beam.sortingOrder = MapLayers.Victims;

            var labelObject = new GameObject("Tag");
            labelObject.transform.SetParent(marker, false);
            labelObject.transform.localPosition = new Vector3(DiscRadiusM + LabelGapM, FloorLiftM, 0f);
            var label = labelObject.AddComponent<TextMesh>();
            label.text = $"V{id}";
            label.font = MapFont.Get();
            label.fontSize = LabelFontSize;
            label.characterSize = LabelCharacterSizeM;
            label.fontStyle = FontStyle.Bold;
            label.anchor = TextAnchor.MiddleLeft;
            label.color = OnMap(VictimRed);
            labelObject.GetComponent<MeshRenderer>().sharedMaterial = label.font.material;
            labelObject.SetActive(showLabels);
            labels.Add(label);
            return marker;
        }
    }
}
