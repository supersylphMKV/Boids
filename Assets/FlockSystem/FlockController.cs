using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Octagon.FlockSystem
{
    public class FlockController : MonoBehaviour
    {
        public SpawnerInfo[] spawners;
        public int maxSpawnperItteration = 1000;

        NativeArray<BoidData> boids;

        Vector3[] rayDirections = BoidHelper.directions;

        void Start()
        {
            StartCoroutine(SpawnEntities());
        }

        // Update is called once per frame
        void Update()
        {
            for(int b = 0; b < boids.Length; b++)
            {
                BoidData boid = boids[b];
                if(IsHeadingForCollision(boid.position, boid.boundsRadius, boid.forward, boid.collisionAvoidDst, boid.layerMask))
                {
                    boid.collAvoidDir = boid.forward;
                }

                boids[b] = boid;
            }
        }

        private void OnDestroy()
        {
            if(boids != null)
            {
                boids.Dispose();
            }
        }

        private void OnDrawGizmos()
        {
            foreach(SpawnerInfo s in spawners)
            {
                if (s.showGizmo)
                {
                    Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
                    Gizmos.DrawSphere(s.spawner.position, s.spawnRadius);
                }
            }
        }

        bool IsHeadingForCollision(Vector3 position, float boundsRadius, Vector3 forward, float collisionAvoidDst, int obstacleMask)
        {
            if (Physics.SphereCast(position, boundsRadius, forward, out _, collisionAvoidDst, obstacleMask))
            {
                return true;
            }
            else { }
            return false;
        }

        IEnumerator SpawnEntities()
        {
            RaycastHit hitInfo;
            int maxSpawn = maxSpawnperItteration;

            List<BoidData> boidDatas = new List<BoidData>();

            foreach (SpawnerInfo s in spawners)
            {
                for (int b = 0; b < s.population; b++)
                {
                    Vector3 pos = s.spawner.position + (UnityEngine.Random.insideUnitSphere * s.spawnRadius);
                    Vector3 dir = UnityEngine.Random.insideUnitSphere;

                    bool isValidPos = false;

                    while (!isValidPos)
                    {
                        if (Physics.Linecast(s.spawner.position, pos, out hitInfo, s.obstacleMask))
                        {
                            pos = hitInfo.point + ((hitInfo.point - pos).normalized);
                        }

                        isValidPos = true;
                    }

                    GameObject go = Instantiate(s.prefab, pos, Quaternion.identity, s.spawner);

                    go.transform.LookAt(pos + dir);

                    boidDatas.Add(new BoidData
                    {
                        boundsRadius = s.settings.boundsRadius,
                        collisionAvoidDst = s.settings.collisionAvoidDst,
                        layerMask = s.obstacleMask,
                        position = pos,
                        direction = dir,
                        rotation = go.transform.rotation,
                    });

                    maxSpawn--;

                    if(maxSpawn == 0)
                    {
                        maxSpawn = maxSpawnperItteration;
                        yield return new WaitForEndOfFrame();
                    }
                }
            }

            boids = new NativeArray<BoidData>(boidDatas.ToArray(), Allocator.Persistent);


        }
    }

    public struct CalculationMovementJob : IJobParallelFor
    {
        NativeArray<BoidData> boids;

        float deltaTime;

        float viewRadius;
        float avoidRadius;

        float alignWeight;
        float cohesionWeight;
        float seperateWeight;
        float maxSpeed;
        float maxSteerForce;

        public void Execute(int index)
        {
            int numFlockMates = 0;
            
            BoidData boida = boids[index];

            for (int i = 0; i < boids.Length; i++)
            {
                if(index != i)
                {
                    BoidData boidb = boids[i];

                    float3 offset = boidb.position - boida.position;
                    float sqrDst = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;

                    if(sqrDst < viewRadius * viewRadius)
                    {
                        numFlockMates++;
                        boida.flockHeading += boidb.direction;
                        boida.flockCenter += boidb.position;

                        if(sqrDst < avoidRadius * avoidRadius)
                        {
                            boida.separationHeading -= offset / sqrDst;
                        }
                    }
                }
            }

            float3 acceleration = float3.zero;

            if(numFlockMates > 0)
            {
                boida.flockCenter /= numFlockMates;

                float3 offsetToFlockmatesCentre = (boida.flockCenter - boida.position);

                var alignmentForce = SteerTowards(boida.flockHeading, boida.velocity) * alignWeight;
                var cohesionForce = SteerTowards(offsetToFlockmatesCentre, boida.velocity) * cohesionWeight;
                var seperationForce = SteerTowards(boida.separationHeading, boida.velocity) * seperateWeight;

                acceleration += alignmentForce;
                acceleration += cohesionForce;
                acceleration += seperationForce;
            }

            boida.acceleration = acceleration;

            boids[index] = boida;
        }

        float3 SteerTowards(float3 vector, float3 velocity)
        {
            float3 v = math.normalize(vector) * maxSpeed - velocity;
            return math.clamp(v, 0, maxSteerForce);
            //return Vector3.ClampMagnitude(v, maxSteerForce);
        }
    }

    [System.Serializable]
    public struct SpawnerInfo
    {
        public GameObject prefab;
        public Transform spawner;
        public float spawnRadius;
        public int population;
        public BoidSettings settings;
        public bool showGizmo;
        public LayerMask obstacleMask;
    }

    public struct BoidData
    {
        public float boundsRadius;
        public float collisionAvoidDst;
        public int layerMask;

        public float3 forward;
        public float3 position;
        public float3 direction;
        public float3 velocity;
        public float3 acceleration;
        public quaternion rotation;

        public float3 flockHeading;
        public float3 flockCenter;
        public float3 separationHeading;
        public int flockMates;

        public float3 collAvoidDir;
        public float3 collAvoidForce;

        public float3 tgtPos;
    }
}

