using UnityEngine;
public abstract class Slot : MonoBehaviour
{
    protected Vector2Int m_coords;
    protected SlotElementData m_slotElementData;
    protected bool m_isActive = true;

    public SlotElementData SlotElementData => m_slotElementData;
    public Vector2Int Coord => m_coords;

    public bool IsActive => m_isActive;

    public virtual void Configure(Vector2Int coords)
    {
        m_coords = coords;
    }

    public virtual void OccupySlot(SlotElementData data)
    {
        m_slotElementData = data;
    }

    public abstract void EmptySlot();
    public abstract bool IsEmpty();

}