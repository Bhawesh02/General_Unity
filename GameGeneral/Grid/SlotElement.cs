using System.Collections.Generic;
using UnityEngine;

public abstract class SlotElement : MonoBehaviour
{
    protected List<Vector2Int> m_occupiedSlots = new List<Vector2Int>();
    protected SlotElementData m_slotElementData;
    protected ItemSlot m_baseItemSlot;
    protected Grid m_grid;
    protected Vector3 m_positionOnSlot;
    
    public Slot BaseItemSlot => m_baseItemSlot;
    public SlotElementData Data => m_slotElementData;
    
    public virtual void Configure(Slot slot, SlotElementData data, Grid grid)
    {
        m_grid = grid;
        m_slotElementData = data;
        m_baseItemSlot = (ItemSlot)slot;
        transform.parent = slot.transform;
        transform.localPosition = data.positionOnSlot;
    }

    public virtual void Configure(SlotElementData data, Grid grid)
    {
        m_slotElementData = data;
        m_grid = grid;
    }
    
    public virtual void Configure(Grid grid)
    {
        m_grid = grid;
    }

}