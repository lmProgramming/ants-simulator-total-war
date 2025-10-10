using Stella3D;
using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

public enum AntScanState
{
    NoScan,
    ScanForFoodPheromone,
    ScanForPathPheromone
}

public sealed class IAntUpdateJobsManager
{
    [Unity.Burst.BurstCompile]
    private struct UpdatePosJob : IJobParallelForTransform
    {
        public NativeArray<float2> positionsArray;
        public NativeArray<float2> velocityArray;
        public NativeArray<float2> desiredDirectionArray;
        [ReadOnly]
        public NativeArray<float> wanderStrengthArray;
        [ReadOnly]
        public NativeArray<float> maxSpeedArray;
        [ReadOnly]
        public NativeArray<float> steerStrengthArray;
        [ReadOnly]
        public NativeArray<int> civIndexes;

        // 0 for not update, 1 for normal, 2 for full force
        [ReadOnly]
        public NativeArray<int> fullForceArray;
        //public NativeArray<Quaternion> rotationArray;

        [ReadOnly]
        public NativeArray<float3> pheromonesArray;
        [ReadOnly]
        public NativeArray<TileState> tileStates;
        [ReadOnly]
        public NativeArray<AntScanState> antScanStates;
        [ReadOnly]
        public NativeArray<int> colonyOnTileCivIndex;
        [ReadOnly]
        public NativeArray<bool> nearWall;

        [WriteOnly]
        public NativeArray<SensorResultType> antScanOutputs;
        [WriteOnly]
        public NativeArray<Vector2Int> antScanTileResults;
        [ReadOnly]
        public NativeArray<int> civIndexOfAntOnTiles;

        [ReadOnly]
        public Unity.Mathematics.Random random;

        [ReadOnly]
        public float deltaTime;

        [ReadOnly]
        public int mapWidth;
        [ReadOnly]
        public int mapHeight;
        [ReadOnly]
        public int mapWidthHalf;
        [ReadOnly]
        public int mapHeightHalf;
        [ReadOnly]
        public int arraySize;

        [ReadOnly] private static readonly int FOUND_GOAL = -6969;
        [ReadOnly] private static readonly int RAY_LENGTH = 12;
        [ReadOnly] private static readonly int RAY_LENGTH_SQUARED = RAY_LENGTH * RAY_LENGTH;
        [ReadOnly] private static readonly float PI_2 = math.PI * 2f;
        [ReadOnly] private static readonly float MINIMUM_SCAN = 0.05f;
        [ReadOnly] private static readonly float FIELD_OF_VISION = 60f;

        //[NativeDisableParallelForRestriction]
        //public NativeArray<int> tempTest;

        // The code actually running on the job
        public void Execute(int antIndex, TransformAccess transform)
        {
            if (antScanStates[antIndex] != AntScanState.NoScan)
            {
                Scan(antIndex, transform);
            }
            else
            {
                float zAngle = transform.rotation.eulerAngles.z;

                Vector2Int antGridPos = GridPos(transform.position);

                antScanOutputs[antIndex] = SensorResultType.Nothing;
                CheckWallInFront(zAngle, transform.position, antGridPos, antIndex);
            }

            if (fullForceArray[antIndex] == 1)
            {
                desiredDirectionArray[antIndex] = math.normalize(desiredDirectionArray[antIndex] + RandomInsideUnitCircle() * wanderStrengthArray[antIndex]);

                float2 desiredVelocity = desiredDirectionArray[antIndex] * maxSpeedArray[antIndex];
                float2 desiredSteeringForce = (desiredVelocity - velocityArray[antIndex]) * steerStrengthArray[antIndex];
                float2 acceleration = ClampMagnitude(desiredSteeringForce, steerStrengthArray[antIndex]);

                velocityArray[antIndex] = ClampMagnitude(velocityArray[antIndex] + acceleration * deltaTime, maxSpeedArray[antIndex]);
                positionsArray[antIndex] += velocityArray[antIndex] * deltaTime;

                transform.position = (Vector2)positionsArray[antIndex];

                transform.rotation = Quaternion.Euler(0, 0, Mathf.Atan2(velocityArray[antIndex].y, velocityArray[antIndex].x) * Mathf.Rad2Deg);
            }
            else if (fullForceArray[antIndex] == 2)
            {
                //desiredDirectionArray[i] *= maxSpeedArray[i];
                float2 desiredVelocity = desiredDirectionArray[antIndex] * maxSpeedArray[antIndex];
                float2 desiredSteeringForce = (desiredVelocity - velocityArray[antIndex]) * steerStrengthArray[antIndex] * 10;
                float2 acceleration = ClampMagnitude(desiredSteeringForce, steerStrengthArray[antIndex]);

                velocityArray[antIndex] = ClampMagnitude(velocityArray[antIndex] + acceleration * deltaTime, maxSpeedArray[antIndex]);
                positionsArray[antIndex] += velocityArray[antIndex] * deltaTime;

                transform.position = (Vector2)positionsArray[antIndex];

                transform.rotation = Quaternion.Euler(0, 0, Mathf.Atan2(velocityArray[antIndex].y, velocityArray[antIndex].x) * Mathf.Rad2Deg);
            }
        }

        private float2 RandomInsideUnitCircle()
        {
            // Generate random angle
            float angle = random.NextFloat(0f, PI_2);

            // Generate random radius
            float radius = math.sqrt(random.NextFloat(0f, 1f));

            // Calculate x and y coordinates
            float x = math.cos(angle) * radius;
            float y = math.sin(angle) * radius;

            return new float2(x, y);
        }

        public void Scan(int antIndex, TransformAccess transform)
        {
            bool checkForFood = antScanStates[antIndex] == AntScanState.ScanForFoodPheromone;

            // path - 0, food - 1
            int pheromoneCheckIndex = checkForFood ? 1 : 0;

            int civIndex = civIndexes[antIndex];

            float zAngle = transform.rotation.eulerAngles.z;

            Vector2Int antGridPos = GridPos(transform.position);

            if (CheckWallInFront(zAngle, transform.position, antGridPos, antIndex) == FOUND_GOAL)
            {
                return;
            }

            int civIndexBias = civIndex * arraySize;

            float topScan = -1;
            Vector2Int chosenDesiredPosition = new Vector2Int(0, 0);

            for (int i = 0; i < 9; i++)
            {
                Vector2Int endPosition = new Vector2Int(0, 0);
                float scan = ScanRay(antGridPos, zAngle + random.NextFloat(-FIELD_OF_VISION, FIELD_OF_VISION), RAY_LENGTH, civIndex, pheromoneCheckIndex, checkForFood, antIndex, ref endPosition, civIndexBias);

                if (scan == FOUND_GOAL)
                {
                    return;
                }
                
                if (scan > topScan)
                {
                    topScan = scan;
                    chosenDesiredPosition = endPosition;
                }
            }

            if (topScan > MINIMUM_SCAN)
            {
                antScanOutputs[antIndex] = SensorResultType.Pheromone;
                antScanTileResults[antIndex] = chosenDesiredPosition;
                //Debug.Log(3 + " " + endPos3);
                return;
            }
            else if (topScan < 0)
            {
                //Debug.Log("Turn back!");
                antScanOutputs[antIndex] = SensorResultType.TurnBack;
                return;
            }

            antScanOutputs[antIndex] = SensorResultType.Nothing;
            return;
        }

        public Vector2Int GridPos(float2 position)
        {
            return new Vector2Int((int)(position.x + mapWidthHalf), (int)(position.y + mapHeightHalf));
        }

        public Vector2Int GridPos(Vector3 position)
        {
            return new Vector2Int((int)(position.x + mapWidthHalf), (int)(position.y + mapHeightHalf));
        }

        public int GridIndex(float2 position)
        {
            return (int)(position.x + mapWidthHalf) + (int)((position.y + mapHeightHalf) * mapWidth);
        }

        public static float2 ClampMagnitude(float2 vector, float maxLength)
        {
            float sqrMagnitude = vector.x * vector.x + vector.y * vector.y;
            if (sqrMagnitude > maxLength * maxLength)
            {
                float mag = (float)Mathf.Sqrt(sqrMagnitude);

                float normalized_x = vector.x / mag;
                float normalized_y = vector.y / mag;
                return new float2(normalized_x * maxLength,
                    normalized_y * maxLength);
            }
            return vector;
        }

        public int CheckWallInFront(float zAngle, Vector2 position, Vector2Int gridPosition, int antIndex)
        {
            int index = gridPosition.x + gridPosition.y * mapWidth;

            // right
            if (zAngle <= 45 || zAngle >= 315)
            {
                //Debug.Log(zAngle + " " + 1);

                int tmpIndex = index + 1;

                if (tileStates[tmpIndex] == TileState.Wall)
                {
                    if (tileStates[tmpIndex + mapWidth] == TileState.Wall)
                    {
                        if (tileStates[tmpIndex - mapWidth] == TileState.Wall)
                        {
                            antScanOutputs[antIndex] = SensorResultType.TurnBack;
                            return FOUND_GOAL;
                        }
                        else
                        {
                            antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                            antScanTileResults[antIndex] = gridPosition + new Vector2Int(1, -2);
                            desiredDirectionArray[antIndex] = position + new Vector2(1, -2);
                            return FOUND_GOAL;
                        }
                    }
                    else if (tileStates[tmpIndex - mapWidth] == TileState.Wall)
                    {
                        antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                        antScanTileResults[antIndex] = gridPosition + new Vector2Int(1, 2);
                        desiredDirectionArray[antIndex] = position + new Vector2(1, 2);
                        return FOUND_GOAL;
                    }
                }
                return 0;
            }
            // up
            else if (zAngle <= 135)
            {
                //Debug.Log(zAngle + " " + 2);

                int tmpIndex = index + mapWidth;
                if (tileStates[tmpIndex] == TileState.Wall)
                {
                    if (tileStates[tmpIndex + 1] == TileState.Wall)
                    {
                        if (tileStates[tmpIndex - 1] == TileState.Wall)
                        {
                            antScanOutputs[antIndex] = SensorResultType.TurnBack;
                            return FOUND_GOAL;
                        }
                        else
                        {
                            antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                            antScanTileResults[antIndex] = gridPosition + new Vector2Int(-2, 1);
                            desiredDirectionArray[antIndex] = position + new Vector2(-2, 1);
                            return FOUND_GOAL;
                        }
                    }
                    else if (tileStates[tmpIndex - 1] == TileState.Wall)
                    {
                        antScanOutputs[antIndex] = SensorResultType.Pheromone;
                        antScanTileResults[antIndex] = gridPosition + new Vector2Int(2, 1);
                        desiredDirectionArray[antIndex] = position + new Vector2(2, 1);
                        return FOUND_GOAL;
                    }
                }
                return 0;
            }
            // left
            else if (zAngle <= 225)
            {
                //Debug.Log(zAngle + " " + 3);

                int tmpIndex = index - 1;

                if (tileStates[tmpIndex] == TileState.Wall)
                {
                    if (tileStates[tmpIndex + mapWidth] == TileState.Wall)
                    {
                        if (tileStates[tmpIndex - mapWidth] == TileState.Wall)
                        {
                            antScanOutputs[antIndex] = SensorResultType.TurnBack;
                            return FOUND_GOAL;
                        }
                        else
                        {
                            antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                            antScanTileResults[antIndex] = gridPosition + new Vector2Int(-1, -2);
                            desiredDirectionArray[antIndex] = position + new Vector2(-1, -2);
                            return FOUND_GOAL;
                        }
                    }
                    else if (tileStates[tmpIndex - mapWidth] == TileState.Wall)
                    {
                        antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                        antScanTileResults[antIndex] = gridPosition + new Vector2Int(-1, 2);
                        desiredDirectionArray[antIndex] = position + new Vector2(-1, 2);
                        return FOUND_GOAL;
                    }
                }
                return 0;
            }
            // down
            else
            {
                //Debug.Log(zAngle + " " + 4);

                int tmpIndex = index - mapWidth;

                if (tileStates[tmpIndex] == TileState.Wall)
                {
                    if (tileStates[tmpIndex + 1] == TileState.Wall)
                    {
                        if (tileStates[tmpIndex - 1] == TileState.Wall)
                        {
                            antScanOutputs[antIndex] = SensorResultType.TurnBack;
                            return FOUND_GOAL;
                        }
                        else
                        {
                            antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                            antScanTileResults[antIndex] = gridPosition + new Vector2Int(-2, -1);
                            desiredDirectionArray[antIndex] = position + new Vector2(-2, -1);
                            return FOUND_GOAL;
                        }
                    }
                    else if (tileStates[tmpIndex - 1] == TileState.Wall)
                    {
                        antScanOutputs[antIndex] = SensorResultType.AvoidObstacle;
                        antScanTileResults[antIndex] = gridPosition + new Vector2Int(2, -1);
                        desiredDirectionArray[antIndex] = position + new Vector2(2, -1);
                        return FOUND_GOAL;
                    }
                }
                return 0;
            }
        }

        public float ScanRay(Vector2Int startGridPos, double angle, int tilesToVisit, int civIndex, int pheromoneCheckIndex, bool checkForFood, int antIndex, ref Vector2Int endPos, int civIndexPheromonesBias)
        {
            double radians = math.radians(angle);
            int dx = (int)(Math.Cos(radians) * tilesToVisit);
            int dy = (int)(Math.Sin(radians) * tilesToVisit);

            int x = startGridPos.x;
            int y = startGridPos.y;

            int endX = x + dx;
            int endY = y + dy;
            endPos = new Vector2Int(endX, endY);

            int stepX = Math.Sign(dx);
            int stepY = Math.Sign(dy);

            dx = Math.Abs(dx);
            dy = Math.Abs(dy);

            int dx2 = dx << 1;
            int dy2 = dy << 1;

            float pheromoneScanned = 0;

            //int skipped = 2;
            if (dx > dy)
            {
                int fraction = dy2 - dx;

                while (x != endX || y != endY)
                {
                    if (fraction >= 0)
                    {
                        y += stepY;
                        fraction -= dx2;
                    }
                    x += stepX;
                    fraction += dy2;

                    int sensorPositionIndex = x + mapWidth * y;

                    if (sensorPositionIndex <= 0 || sensorPositionIndex >= arraySize)
                    {
                        // SHOULD NEVER HAPPEN
                        return pheromoneScanned;
                    }

                    //tempTest[sensorPositionIndex] = 1;

                    int colonyCivIndex = colonyOnTileCivIndex[sensorPositionIndex];
                    if (checkForFood)
                    {
                        if (tileStates[sensorPositionIndex] == TileState.Food)
                        {
                            antScanOutputs[antIndex] = SensorResultType.Food;
                            antScanTileResults[antIndex] = new Vector2Int(x, y);
                            return FOUND_GOAL;
                        }
                    }
                    else
                    {
                        if (colonyCivIndex == civIndex)
                        {
                            antScanOutputs[antIndex] = SensorResultType.Colony;
                            antScanTileResults[antIndex] = new Vector2Int(x, y);
                            return FOUND_GOAL;
                        }
                    }

                    if (colonyCivIndex != -1 && colonyCivIndex != civIndex)
                    {
                        antScanOutputs[antIndex] = SensorResultType.Colony;
                        antScanTileResults[antIndex] = new Vector2Int(x, y);
                        return FOUND_GOAL;
                    }

                    if (civIndexOfAntOnTiles[sensorPositionIndex] != -1 && civIndexOfAntOnTiles[sensorPositionIndex] != civIndex)
                    {
                        antScanOutputs[antIndex] = SensorResultType.EnemyAnt;
                        antScanTileResults[antIndex] = new Vector2Int(x, y);
                        return FOUND_GOAL;
                    }

                    if (nearWall[sensorPositionIndex])
                    {
                        float lengthSquared = math.pow(startGridPos.x - x, 2) + math.pow(startGridPos.y - y, 2);
                        float mult = lengthSquared / RAY_LENGTH_SQUARED;

                        if (mult > 0.75f)
                        {
                            return -1;
                        }

                        if (mult > 0.5f)
                        {
                            pheromoneScanned -= 1;
                        }
                        return pheromoneScanned;
                    }

                    pheromoneScanned += pheromonesArray[civIndexPheromonesBias + sensorPositionIndex][pheromoneCheckIndex];

                    tilesToVisit--;
                }
            }
            else
            {
                int fraction = dx2 - dy;

                while (x != endX || y != endY)
                {
                    if (fraction >= 0)
                    {
                        x += stepX;
                        fraction -= dy2;
                    }
                    y += stepY;
                    fraction += dx2;

                    int sensorPositionIndex = x + mapWidth * y;

                    if (sensorPositionIndex <= 0 || sensorPositionIndex >= arraySize)
                    {
                        // SHOULD NEVER HAPPEN
                        return pheromoneScanned;
                    }

                    // tempTest[sensorPositionIndex] = 1;

                    int colonyCivIndex = colonyOnTileCivIndex[sensorPositionIndex];
                    if (checkForFood)
                    {
                        if (tileStates[sensorPositionIndex] == TileState.Food)
                        {
                            antScanOutputs[antIndex] = SensorResultType.Food;
                            antScanTileResults[antIndex] = new Vector2Int(x, y);
                            return FOUND_GOAL;
                        }
                    }
                    else
                    {
                        if (colonyCivIndex == civIndex)
                        {
                            antScanOutputs[antIndex] = SensorResultType.Colony;
                            antScanTileResults[antIndex] = new Vector2Int(x, y);
                            return FOUND_GOAL;
                        }
                    }

                    if (colonyCivIndex != -1 && colonyCivIndex != civIndex)
                    {
                        antScanOutputs[antIndex] = SensorResultType.Colony;
                        antScanTileResults[antIndex] = new Vector2Int(x, y);
                        return FOUND_GOAL;
                    }

                    if (civIndexOfAntOnTiles[sensorPositionIndex] != -1 && civIndexOfAntOnTiles[sensorPositionIndex] != civIndex)
                    {
                        antScanOutputs[antIndex] = SensorResultType.EnemyAnt;
                        antScanTileResults[antIndex] = new Vector2Int(x, y);
                        return FOUND_GOAL;
                    }

                    if (nearWall[sensorPositionIndex])
                    {
                        float lengthSquared = math.pow(startGridPos.x - x, 2) + math.pow(startGridPos.y - y, 2);
                        float mult = lengthSquared / RAY_LENGTH_SQUARED;

                        if (mult > 0.75f)
                        {
                            return -1;
                        }
                        else if (mult > 0.5f)
                        {
                            pheromoneScanned -= 1;
                        }
                        return pheromoneScanned;
                    }
                    else
                    {
                        pheromoneScanned += pheromonesArray[civIndexPheromonesBias + sensorPositionIndex][pheromoneCheckIndex];
                    }

                    tilesToVisit--;
                }
            }

            return pheromoneScanned;
        }
    }

    public static IAntUpdateJobsManager Instance;

    private IGridTileUpdater gridTileUpdater;

    private SharedArray<Vector2, float2> _positionsArray;
    private Vector2[] _positionsArrayA;

    private SharedArray<Vector2, float2> _velocityArray;
    private Vector2[] _velocityArrayA;

    private SharedArray<Vector2, float2> _desiredDirectionArray;
    private Vector2[] _desiredDirectionArrayA;

    private NativeArray<float> _wanderStrengthArray;
    private NativeArray<float> _maxSpeedArray;
    private NativeArray<float> _steerStrengthArray;

    private SharedArray<int> _fullForceArray;
    public int[] _fullForceArrayA;

    private Transform[] allAntsTransforms;
    private bool needToRegenerateMAccessArray = false;
    private Transform dummyTransform; // Placeholder for null transforms

    private TransformAccessArray m_AccessArray;

    private SharedArray<int> antCivIndexes;
    public int[] antCivIndexesA;
    private NativeArray<int> antCivIndexesNA;

    private SharedArray<AntScanState> antScanStates;
    private AntScanState[] antScanStatesA;
    public NativeArray<AntScanState> antScanStatesNA;

    private SharedArray<SensorResultType> antScanOutputs;
    private SensorResultType[] antScanOutputsA;
    private NativeArray<SensorResultType> antScanOutputsNA;

    private SharedArray<Vector2Int, Vector2Int> antScanTileResults;
    private Vector2Int[] antScanTileResultsA;

    public void InitializeLists(Ant[] allAnts)
    {
        Instance = this;

        gridTileUpdater = IGridTileUpdater.Instance;

        // Create a dummy GameObject with a Transform for placeholder purposes
        GameObject dummyObject = new GameObject("DummyTransformForJobs");
        dummyObject.SetActive(false); // Keep it inactive to avoid rendering or processing
        dummyTransform = dummyObject.transform;
        UnityEngine.Object.DontDestroyOnLoad(dummyObject); // Persist across scene loads if needed

        _positionsArray = new SharedArray<Vector2, float2>(allAnts.Length);
        _velocityArray = new SharedArray<Vector2, float2>(allAnts.Length);
        _desiredDirectionArray = new SharedArray<Vector2, float2>(allAnts.Length);

        _wanderStrengthArray = new NativeArray<float>(allAnts.Length, Allocator.Persistent, NativeArrayOptions.ClearMemory);
        _maxSpeedArray = new NativeArray<float>(allAnts.Length, Allocator.Persistent, NativeArrayOptions.ClearMemory);
        _steerStrengthArray = new NativeArray<float>(allAnts.Length, Allocator.Persistent, NativeArrayOptions.ClearMemory);

        _fullForceArray = new SharedArray<int>(allAnts.Length);

        allAntsTransforms = new Transform[allAnts.Length];

        _positionsArrayA = _positionsArray;

        _velocityArrayA = _velocityArray;

        _desiredDirectionArrayA = _desiredDirectionArray;

        _fullForceArrayA = _fullForceArray;

        antScanStates = new SharedArray<AntScanState>(allAnts.Length);
        antScanStatesA = antScanStates;
        antScanStatesNA = antScanStates;

        antScanOutputs = new SharedArray<SensorResultType>(allAnts.Length);
        antScanOutputsA = antScanOutputs;
        antScanOutputsNA = antScanOutputs;

        antScanTileResults = new SharedArray<Vector2Int>(allAnts.Length);
        antScanTileResultsA = antScanTileResults;

        antCivIndexes = new SharedArray<int>(allAnts.Length);
        antCivIndexesA = antCivIndexes;
        antCivIndexesNA = antCivIndexes;
    }

    public void AssignAntsToJob(Ant[] allAnts)
    {
        for (int i = 0; i < allAnts.Length; i++)
        {
            if (allAnts[i] != null)
            {
                Ant ant = allAnts[i];

                ant.UpdateJobIndex = i;

                UpdateValuesFromAnt(ant);
            }
            else
            {
                _fullForceArrayA[i] = -1;
                allAntsTransforms[i] = dummyTransform; // Use dummy transform instead of null
            }
        }

        m_AccessArray = new TransformAccessArray(allAntsTransforms);

        needToRegenerateMAccessArray = true;
    }

    public void RemoveAntFromList(Ant antToRemove)
    {
        _fullForceArrayA[antToRemove.UpdateJobIndex] = -1;
        allAntsTransforms[antToRemove.UpdateJobIndex] = dummyTransform; // Use dummy transform instead of null

        needToRegenerateMAccessArray = true;
    }

    public void UpdatePositions(Ant[] allAnts)
    {
        if (needToRegenerateMAccessArray)
        {
            m_AccessArray.SetTransforms(allAntsTransforms);
            needToRegenerateMAccessArray = false;
        }

        // Initialize the job data
        var job = new UpdatePosJob()
        {
            deltaTime = GameInput.deltaTime,
            positionsArray = _positionsArray,
            velocityArray = _velocityArray,
            desiredDirectionArray = _desiredDirectionArray,
            wanderStrengthArray = _wanderStrengthArray,
            maxSpeedArray = _maxSpeedArray,
            steerStrengthArray = _steerStrengthArray,
            fullForceArray = _fullForceArray,

            pheromonesArray = gridTileUpdater.pheromonesNA,
            tileStates = gridTileUpdater.tileStates,
            civIndexes = antCivIndexes,
            colonyOnTileCivIndex = gridTileUpdater.colonyOnTileCivIndexNA,
            civIndexOfAntOnTiles = gridTileUpdater.antOnTileCivIndexes,
            nearWall = gridTileUpdater.nearWall,

            antScanStates = antScanStates,
            antScanTileResults = antScanTileResults,
            antScanOutputs = antScanOutputsNA,

            random = new Unity.Mathematics.Random((uint)UnityEngine.Random.Range(1000f, 10000000f)),

            mapWidth = MapGenerator.mapWidth,
            mapHeight = MapGenerator.mapHeight,
            mapHeightHalf = MapGenerator.mapHeightHalf,
            mapWidthHalf = MapGenerator.mapWidthHalf,
            arraySize = MapGenerator.Instance.tiles.Length,

            //tempTest = gridTileUpdater.tempTest,
        };

        JobHandle jobHandle = job.Schedule(m_AccessArray);

        jobHandle.Complete();

        int antIndex;
        for (int i = 0; i < allAnts.Length; i++)
        {
            if (allAnts[i] is not null)
            {
                antIndex = allAnts[i].UpdateJobIndex;

                if (_fullForceArrayA[i] == 0)
                {
                    allAnts[i].antBody.FightingVisual();
                }
                else
                {
                    allAnts[i].antMovement.Position = _positionsArrayA[antIndex];
                }

                if (antScanOutputsA[antIndex] != SensorResultType.Nothing)
                {
                    allAnts[i].antBrain.HandleScanResults(antScanOutputsA[antIndex], antScanTileResultsA[antIndex]);
                }
            }
        }

        //int[] arr = gridTileUpdater.tempTestA;
        //gridTileUpdater.UpdateAllPheromonesSpritesManually(0);
        //for (int i = 0; i < arr.Length; i++)
        //{
        //    if (arr[i] != 0)
        //    {
        //        MapGenerator.Instance.TileAt(new Vector2Int(i % MapGenerator.mapWidth, i / MapGenerator.mapWidth)).ChangeColorToTest();
        //        arr[i] = 0;
        //    }
        //}
    }

    public void UpdateAntDesiredDirection(Ant ant, Vector2 newDesiredDirection)
    {
        _desiredDirectionArrayA[ant.UpdateJobIndex] = newDesiredDirection;
    }

    public void UpdateAntPosition(Ant ant, Vector2 newPosition)
    {
        _positionsArrayA[ant.UpdateJobIndex] = newPosition;
    }

    public void AddAnt(Ant ant)
    {
        UpdateValuesFromAnt(ant);
        needToRegenerateMAccessArray = true;
    }

    public void UpdateAntMovement(Ant ant)
    {
        int i = ant.UpdateJobIndex;

        _positionsArrayA[i] = ant.antBody.Position;
        _velocityArrayA[i] = ant.antMovement.Velocity;
        _desiredDirectionArrayA[i] = ant.antMovement.desiredDirection;
    }

    public void UpdateValuesFromAnt(Ant ant)
    {
        int i = ant.UpdateJobIndex;

        _positionsArrayA[i] = ant.antBody.Position;
        _velocityArrayA[i] = ant.antMovement.Velocity;
        _desiredDirectionArrayA[i] = ant.antMovement.desiredDirection;
        _wanderStrengthArray[i] = ant.antMovement.wanderStrength;
        _maxSpeedArray[i] = ant.antMovement.maxSpeed;
        _steerStrengthArray[i] = ant.antMovement.steerStrength;
        antCivIndexesA[i] = ant.CivIndex;
        allAntsTransforms[i] = ant.transform;
        _fullForceArrayA[i] = 1;
    }

    public void DisposeOfArrays()
    {
        // Native arrays must be disposed manually.
        _positionsArray.Dispose();
        _velocityArray.Dispose();
        _desiredDirectionArray.Dispose();
        _wanderStrengthArray.Dispose();
        _maxSpeedArray.Dispose();
        _steerStrengthArray.Dispose();
        _fullForceArray.Dispose();
        m_AccessArray.Dispose();

        // Clean up the dummy transform GameObject
        if (dummyTransform != null)
        {
            UnityEngine.Object.Destroy(dummyTransform.gameObject);
        }
    }

    public void UpdateAntWanderingStrength(Ant ant)
    {
        _wanderStrengthArray[ant.UpdateJobIndex] = ant.antMovement.wanderStrength;
    }

    public void UpdateAntSpeed(Ant ant, float speed)
    {
        _maxSpeedArray[ant.UpdateJobIndex] = speed;
    }
}
