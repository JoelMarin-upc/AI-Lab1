
using System.Collections.Generic;
using UnityEngine;

public class FlockingBehaviour : MonoBehaviour
{
    [Header("Flock Units")]
    public int unitCount = 0;
    public GameObject unitPrefab = null;

    [Header("Detection")]
    public float AvoidRadius = 2f;
    public float NeighbourRadius = 5f;
    public float ObstacleDetectionDistance = 4f;
    public float ObstacleAvoidanceRadius = 1f;
    public float SideCheckDistance = 2f;

    [Header("Movement")]
    public float MaxSpeed = 5f;
    public float MaxSteeringSpeed = 3f;

    [Header("Flocking Weights")]
    public float SeparationWeight = 1f;
    public float CohesionWeight = 1f;
    public float AlignmentWeight = 1f;
    public float FollowLeaderWeight = 1f;
    public float FollowWaypointWeight = 1f;
    public float EvadeWeight = 1f;
    public float SideSteeringWeight = 1f;
    public float ObstacleAvoidanceWeight = 1f;

    [Header("Environment")]
    public List<Collider> Waypoints = new();
    public List<Collider> Obstacles = new();

    int nextWaypointIndex = 0;
    Collider NextWaypoint => Waypoints.Count == 0 ? null : Waypoints[nextWaypointIndex];

    readonly List<Rigidbody> flockUnits = new();

    Rigidbody leader = null;

    void Start()
    {
        float spawnRadius = NeighbourRadius * 2f;
        int maxAttempts = 100;

        for (int i = 0; i < unitCount; i++)
        {
            var spawnPosition = transform.position;
            bool validPosition = false;

            for (int attempt = 0; attempt < maxAttempts; attempt++)
            {
                var randomOffset = Random.insideUnitSphere * spawnRadius;
                spawnPosition = transform.position + randomOffset;

                validPosition = true;

                foreach (var other in flockUnits)
                {
                    if (other == null) continue;

                    float minDistance = AvoidRadius * 0.1f;

                    if (Vector3.Distance(spawnPosition, other.position) < minDistance)
                    {
                        validPosition = false;
                        break;
                    }
                }

                if (validPosition) break;
            }

            if (!validPosition)
            {
                Debug.LogWarning($"Could not find a valid spawn position for unit {i}.");
                continue;
            }

            GameObject unit = Instantiate(unitPrefab, spawnPosition, Random.rotation, transform);
            
            if (!unit.TryGetComponent<Rigidbody>(out var rb))
            {
                Debug.LogError("The unit prefab must have a Rigidbody component.", unit);
                Destroy(unit);
                continue;
            }

            rb.maxLinearVelocity = MaxSpeed;
            rb.maxAngularVelocity = MaxSteeringSpeed;
            rb.constraints |= RigidbodyConstraints.FreezeRotation;

            flockUnits.Add(rb);
        }
    }

    void FixedUpdate()
    {
        SetLeader();
        List<Vector3> directions = new();

        foreach (var unit in flockUnits)
        {
            if (unit == null)
            {
                directions.Add(Vector3.zero);
                continue;
            }

            var dir = ApplyFlocking(unit);

            directions.Add(dir);
        }

        for (int i = 0; i < flockUnits.Count; i++)
        {
            var unit = flockUnits[i];

            if (unit == null) continue;

            var direction = directions[i];
            if (direction.sqrMagnitude > 0.001f)
            {
                var desiredVelocity = direction.normalized * MaxSpeed;
                var steering = desiredVelocity - unit.linearVelocity;
                steering = Vector3.ClampMagnitude(steering, MaxSteeringSpeed);
                unit.AddForce(steering, ForceMode.Acceleration);
            }

            unit.linearVelocity = Vector3.ClampMagnitude(unit.linearVelocity, MaxSpeed);
        }
    }

    void SetNextWaypoint()
    {
        nextWaypointIndex = (nextWaypointIndex + 1) % Waypoints.Count;
    }

    void SetLeader()
    {
        if (NextWaypoint == null) return;

        float closestDistance = float.MaxValue;
        Rigidbody closestUnit = null;

        foreach (var unit in flockUnits)
        {
            if (unit == null) continue;

            Vector3 closestPoint = NextWaypoint.ClosestPoint(unit.position);
            float distance = Vector3.Distance(unit.position, closestPoint);

            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestUnit = unit;
            }
        }

        if (closestUnit != null) leader = closestUnit;
    }

    Vector3 ApplyFlocking(Rigidbody unit)
    {
        if (NextWaypoint != null && NextWaypoint.ClosestPoint(unit.position) == unit.position) SetNextWaypoint();

        var separation     = ApplySeparation(unit)     * SeparationWeight;
        var cohesion       = ApplyCohesion(unit)       * CohesionWeight;
        var alignment      = ApplyAlignment(unit)      * AlignmentWeight;
        var followLeader   = ApplyFollowLeader(unit)   * FollowLeaderWeight;
        var followWaypoint = ApplyFollowWaypoint(unit) * FollowWaypointWeight;
        var evade          = ApplyEvadeObstacle(unit)  * EvadeWeight;

        var flock = separation + cohesion + alignment + followLeader + followWaypoint;

        float urgency = Mathf.Clamp01(evade.magnitude);
        return flock * (1f - urgency) + evade;
    }

    Vector3 ApplySeparation(Rigidbody unit)
    {
        var separation = Vector3.zero;

        foreach (var other in flockUnits)
        {
            if (other == null || other == unit) continue;

            var offset = unit.position - other.position;
            float distance = offset.magnitude;

            if (distance < AvoidRadius)
            {
                var dir = distance > 0.001f ? offset / distance : Random.onUnitSphere;
                float strength = 1f - distance / AvoidRadius;
                separation += dir * strength;
            }
        }

        return Vector3.ClampMagnitude(separation, 1f);
    }

    Vector3 ApplyCohesion(Rigidbody unit)
    {
        var neighbours = GetNeighbours(unit);

        if (neighbours.Count == 0) return Vector3.zero;

        var center = GetCenterOfMass(neighbours);
        var direction = center - unit.position;

        if (direction.sqrMagnitude > 0.001f) return direction.normalized;
        return Vector3.zero;
    }

    Vector3 ApplyAlignment(Rigidbody unit)
    {
        var neighbours = GetNeighbours(unit);

        if (neighbours.Count == 0) return Vector3.zero;

        var averageVelocity = Vector3.zero;
        foreach (var neighbour in neighbours) averageVelocity += neighbour.linearVelocity;
        averageVelocity /= neighbours.Count;

        if (averageVelocity.sqrMagnitude > 0.001f) return averageVelocity.normalized;
        return Vector3.zero;
    }

    Vector3 ApplyFollowLeader(Rigidbody unit)
    {
        if (leader == null || unit == leader) return Vector3.zero;

        var direction = leader.position - unit.position;

        if (direction.sqrMagnitude > 0.001f) return direction.normalized;
        return Vector3.zero;
    }

    Vector3 ApplyFollowWaypoint(Rigidbody unit)
    {
        if (NextWaypoint == null) return Vector3.zero;

        Vector3 target = NextWaypoint.ClosestPoint(unit.position);
        Vector3 direction = target - unit.position;

        if (direction.sqrMagnitude > 0.001f) return direction.normalized;
        return Vector3.zero;
    }

    Vector3 ApplyEvadeObstacle(Rigidbody unit)
    {
        Vector3 push = Vector3.zero;
        Vector3 velocity = unit.linearVelocity;

        foreach (var obstacle in Obstacles)
        {
            if (obstacle == null) continue;

            Vector3 closest = obstacle.ClosestPoint(unit.position);
            Vector3 away = unit.position - closest;

            float distance = away.magnitude;
            if (distance >= ObstacleDetectionDistance) continue;

            if (distance < 0.001f) away = unit.position - obstacle.bounds.center;
            away.Normalize();
            
            float proximity = 1f - Mathf.Clamp01(distance / ObstacleDetectionDistance);
            Vector3 force = away;
            Vector3 slide = Vector3.ProjectOnPlane(velocity, away);
            if (slide.sqrMagnitude > 0.01f) force += slide.normalized * SideSteeringWeight;

            push += force * proximity;
        }

        return push;
    }

    List<Rigidbody> GetNeighbours(Rigidbody unit)
    {
        var neighbours = new List<Rigidbody>();

        foreach (var other in flockUnits)
        {
            if (other == null || other == unit) continue;

            float distance = Vector3.Distance(unit.position, other.position);
            if (distance < NeighbourRadius) neighbours.Add(other);
        }

        return neighbours;
    }

    Vector3 GetCenterOfMass(List<Rigidbody> group)
    {
        if (group.Count == 0) return Vector3.zero;

        var center = Vector3.zero;
        int count = 0;

        foreach (var unit in group)
        {
            if (unit == null) continue;

            center += unit.position;
            count++;
        }

        return count > 0 ? center / count : Vector3.zero;
    }
}
