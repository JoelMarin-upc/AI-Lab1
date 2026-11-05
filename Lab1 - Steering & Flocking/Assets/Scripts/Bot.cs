using UnityEngine;
using UnityEngine.AI;

public class Bot : MonoBehaviour
{
    NavMeshAgent agent;

    public GameObject target;

    [Header("Behaviour Settings")]
    public BehaviourMode behaviour = BehaviourMode.Automatic;

    public enum BehaviourMode
    {
        Automatic,
        Seek,
        Flee
    }

    [Min(0.1f)]
    [Tooltip("Robber flees when the cop is closer than this.")]
    public float safetyDistance = 10f;

    float EscapeLookahead => Mathf.Max(safetyDistance, 5f);

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
    }

    void Seek(Vector3 location)
    {
        agent.SetDestination(location);
    }

    void Flee(Vector3 location)
    {
        Vector3 awayDirection = transform.position - location;
        awayDirection = Vector3.ProjectOnPlane(awayDirection, Vector3.up);

        if (awayDirection.sqrMagnitude < 0.001f)
        {
            awayDirection = -target.transform.forward;
        }

        awayDirection.Normalize();

        float[] angles = { 0f, 45f, -45f, 90f, -90f, 135f, -135f, 180f };

        NavMeshPath path = new NavMeshPath();

        foreach (float angle in angles)
        {
            Vector3 direction =
                Quaternion.Euler(0f, angle, 0f) * awayDirection;

            Vector3 escapePoint =
                transform.position + direction * EscapeLookahead;

            NavMeshHit hit;

            if (!NavMesh.SamplePosition(
                escapePoint, out hit, 2.5f, agent.areaMask))
            {
                continue;
            }

            if (Vector3.Distance(transform.position, hit.position) < 1.5f)
                continue;

            if (agent.CalculatePath(hit.position, path) &&
                path.status == NavMeshPathStatus.PathComplete)
            {
                agent.SetDestination(hit.position);
                return;
            }
        }

        agent.ResetPath();
    }

    void Update()
    {
        if (target == null || agent == null)
            return;

        float distance = Vector3.Distance(
            transform.position,
            target.transform.position
        );

        switch (behaviour)
        {
            case BehaviourMode.Seek:
                Seek(target.transform.position);
                break;

            case BehaviourMode.Flee:
                Flee(target.transform.position);
                break;

            case BehaviourMode.Automatic:
                if (distance > safetyDistance)
                    Seek(target.transform.position);
                else
                    Flee(target.transform.position);
                break;
        }
    }
}