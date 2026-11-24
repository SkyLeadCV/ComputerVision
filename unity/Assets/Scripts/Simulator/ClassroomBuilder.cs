using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>The materials of the virtual classroom. Any of them can be null, as in tests that only need the geometry.</summary>
    public struct ClassroomLook
    {
        public Material Floor;
        public Material Wall;
        public Material Tables;
        public Material VictimClothes;
        public Material RescuerClothes;
        public Material Skin;
    }

    /// <summary>
    /// Builds the classroom of <see cref="ClassroomLayout"/> out of primitives. Every solid part keeps the collider its
    /// primitive comes with, so the simulated drone can detect collisions with walls, tables and people.
    /// </summary>
    public static class ClassroomBuilder
    {
        const float WallThicknessCm = 10f;

        const float BodyLengthCm = 150f;
        const float BodyWidthCm = 40f;
        const float BodyThicknessCm = 25f;
        const float StandingBodyHeightCm = 150f;
        const float HeadCm = 22f;

        public static GameObject Build(ClassroomLook look)
        {
            var room = new GameObject("Classroom");
            BuildFloor(room.transform, look.Floor);

            // The walls stand outside the room's outline, so its inside is exactly the measured plan.
            float w = WallThicknessCm, width = ClassroomLayout.WidthCm, depth = ClassroomLayout.DepthCm;
            float ceiling = ClassroomLayout.CeilingCm;
            Box("Wall Left", new Vector3(-w, -w, 0f), new Vector3(0f, depth + w, ceiling), look.Wall, room.transform);
            Box("Wall Right", new Vector3(width, -w, 0f), new Vector3(width + w, depth + w, ceiling), look.Wall, room.transform);
            Box("Wall Bottom", new Vector3(0f, -w, 0f), new Vector3(width, 0f, ceiling), look.Wall, room.transform);
            Box("Wall Top", new Vector3(0f, depth, 0f), new Vector3(width, depth + w, ceiling), look.Wall, room.transform);

            Vector2 tablesMin = ClassroomLayout.TablesMinCm, tablesMax = ClassroomLayout.TablesMaxCm;
            Box("Tables Block", new Vector3(tablesMin.x, tablesMin.y, 0f), new Vector3(tablesMax.x, tablesMax.y, ceiling), look.Tables, room.transform);

            for (int i = 0; i < ClassroomLayout.Victims.Length; i++)
                LyingPerson($"Victim {i + 1}", ClassroomLayout.Victims[i], look, room.transform);
            StandingPerson("Rescuer", ClassroomLayout.Rescuer, look, room.transform);
            return room;
        }

        static void BuildFloor(Transform parent, Material material)
        {
            var floor = GameObject.CreatePrimitive(PrimitiveType.Quad);
            floor.name = "Floor";
            floor.transform.SetParent(parent, false);
            float width = ClassroomLayout.WidthCm, depth = ClassroomLayout.DepthCm;
            floor.transform.localPosition = ClassroomLayout.ToUnity(width / 2f, depth / 2f, 0f);
            // A quad faces -z; tilting it forward makes it face up with its texture's v along the room's +y.
            floor.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
            floor.transform.localScale = new Vector3(width, depth, 100f) / 100f;
            floor.GetComponent<Renderer>().sharedMaterial = material;
        }

        /// <summary>An axis-aligned box between two room corners in cm.</summary>
        static GameObject Box(string name, Vector3 minCm, Vector3 maxCm, Material material, Transform parent)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent, false);
            Vector3 centre = (minCm + maxCm) / 2f, size = maxCm - minCm;
            box.transform.localPosition = ClassroomLayout.ToUnity(centre.x, centre.y, centre.z);
            box.transform.localScale = ClassroomLayout.ToUnity(size.x, size.y, size.z);
            box.GetComponent<Renderer>().sharedMaterial = material;
            return box;
        }

        /// <summary>A person lying on the floor with the head towards the heading.</summary>
        static void LyingPerson(string name, PersonPlacement placement, ClassroomLook look, Transform parent)
        {
            Transform person = Person(name, placement, parent);
            // A capsule's long axis is its local y; lying down puts it along the person's forward axis.
            Part(PrimitiveType.Capsule, "Body", person, look.VictimClothes,
                new Vector3(0f, BodyThicknessCm / 2f, 0f), Quaternion.Euler(90f, 0f, 0f),
                new Vector3(BodyWidthCm, BodyLengthCm / 2f, BodyThicknessCm));
            Part(PrimitiveType.Sphere, "Head", person, look.Skin,
                new Vector3(0f, HeadCm / 2f, (BodyLengthCm + HeadCm) / 2f), Quaternion.identity, Vector3.one * HeadCm);
        }

        static void StandingPerson(string name, PersonPlacement placement, ClassroomLook look, Transform parent)
        {
            Transform person = Person(name, placement, parent);
            Part(PrimitiveType.Capsule, "Body", person, look.RescuerClothes,
                new Vector3(0f, StandingBodyHeightCm / 2f, 0f), Quaternion.identity,
                new Vector3(BodyWidthCm, StandingBodyHeightCm / 2f, BodyThicknessCm));
            Part(PrimitiveType.Sphere, "Head", person, look.Skin,
                new Vector3(0f, StandingBodyHeightCm + HeadCm / 2f, 0f), Quaternion.identity, Vector3.one * HeadCm);
        }

        static Transform Person(string name, PersonPlacement placement, Transform parent)
        {
            var person = new GameObject(name).transform;
            person.SetParent(parent, false);
            person.localPosition = ClassroomLayout.ToUnity(placement.PositionCm.x, placement.PositionCm.y, 0f);
            person.localRotation = ClassroomLayout.HeadingToUnity(placement.HeadingDeg);
            return person;
        }

        /// <summary>A body part with its position and scale in cm, in the person's own axes (Unity's, y up).</summary>
        static void Part(PrimitiveType shape, string name, Transform person, Material material, Vector3 positionCm, Quaternion rotation, Vector3 scaleCm)
        {
            var part = GameObject.CreatePrimitive(shape);
            part.name = name;
            part.transform.SetParent(person, false);
            part.transform.localPosition = positionCm / 100f;
            part.transform.localRotation = rotation;
            part.transform.localScale = scaleCm / 100f;
            part.GetComponent<Renderer>().sharedMaterial = material;
        }
    }
}
