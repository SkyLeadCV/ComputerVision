using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>A red beacon for every victim: a glow on the floor, a pulsing ring, a light beam and its tag.
    /// A repeated id moves its beacon (docs/protocol.md).</summary>
    public sealed class VictimMarkers : MonoBehaviour
    {
        [Tooltip("Bright vertex-coloured material: ring and beam.")]
        [SerializeField] Material glowMaterial;

        [Tooltip("Faint vertex-coloured material: the glow on the floor.")]
        [SerializeField] Material holoMaterial;

        // Taller than the tables (about 75 cm), so the beam shows from the 3D view over the furniture.
        const float BeamHeightM = 1.4f;
        const float BeamWidthM = 0.03f;
        const float RingWidthM = 0.02f;
        // About a person's shoulder width, so the glow covers someone lying on the floor.
        const float GlowRadiusM = 0.45f;
        const float PulseMinRadiusM = 0.15f;
        const float PulseMaxRadiusM = 0.6f;
        const float PulsePeriodS = 1.6f;
        const float StillRingRadiusM = 0.35f;
        const float FloorLiftM = 0.012f;
        const float LabelGapM = 0.08f;
        const float LabelCharacterSizeM = 0.045f;
        const int LabelFontSize = 64;

        static readonly Color Red = new(1f, 0.1f, 0.08f);

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
            // Each ring grows and fades out, like a beacon, so a victim catches the eye on a busy map.
            float phase = Time.time / PulsePeriodS % 1f;
            float radius = Mathf.Lerp(PulseMinRadiusM, PulseMaxRadiusM, phase);
            var color = new Color(Red.r, Red.g, Red.b, 1f - phase);
            foreach (LineRenderer ring in rings)
            {
                ring.transform.localScale = Vector3.one * radius;
                ring.startColor = ring.endColor = color;
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

            MapShapes.MeshObject(marker, "Glow", MapShapes.Disc(GlowRadiusM, new Color(Red.r, Red.g, Red.b, 0.55f),
                new Color(Red.r, Red.g, Red.b, 0f)), holoMaterial).transform.localPosition = lift;

            LineRenderer outline = MapShapes.Line(marker, "Outline", glowMaterial, RingWidthM, true);
            outline.transform.localPosition = lift;
            MapShapes.SetCircle(outline, StillRingRadiusM);
            outline.startColor = outline.endColor = Red;

            // A unit circle scaled by the pulse; the line's width does not scale with it.
            LineRenderer ring = MapShapes.Line(marker, "Pulse", glowMaterial, RingWidthM, true);
            ring.transform.localPosition = lift;
            MapShapes.SetCircle(ring, 1f);
            ring.transform.localScale = Vector3.one * StillRingRadiusM;
            ring.startColor = ring.endColor = new Color(Red.r, Red.g, Red.b, 0f);
            rings.Add(ring);

            LineRenderer beam = MapShapes.Line(marker, "Beam", glowMaterial, BeamWidthM, false);
            beam.positionCount = 2;
            beam.SetPosition(0, Vector3.zero);
            beam.SetPosition(1, new Vector3(0f, BeamHeightM, 0f));
            beam.startColor = Red;
            beam.endColor = new Color(Red.r, Red.g, Red.b, 0f);

            var labelObject = new GameObject("Tag");
            labelObject.transform.SetParent(marker, false);
            labelObject.transform.localPosition = new Vector3(0f, BeamHeightM + LabelGapM, 0f);
            var label = labelObject.AddComponent<TextMesh>();
            label.text = $"V{id}";
            label.font = MapFont.Get();
            label.fontSize = LabelFontSize;
            label.characterSize = LabelCharacterSizeM;
            label.fontStyle = FontStyle.Bold;
            label.anchor = TextAnchor.LowerCenter;
            label.color = new Color(1f, 0.85f, 0.85f);
            labelObject.GetComponent<MeshRenderer>().sharedMaterial = label.font.material;
            labelObject.SetActive(showLabels);
            labels.Add(label);
            return marker;
        }
    }
}
