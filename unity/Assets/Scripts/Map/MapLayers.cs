namespace SkyLead.Map
{
    /// <summary>Drawing order of the map's flat, transparent layers, from the floor up.</summary>
    public static class MapLayers
    {
        public const int Unexplored = -1;
        public const int Floor = 0;
        public const int Grid = 1;
        public const int Walls = 2;
        public const int Scan = 3;
        public const int Trail = 4;
        public const int Drone = 5;
        public const int Victims = 6;
    }
}
