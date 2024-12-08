public abstract class ItemSlot : Slot
{
    protected SlotElement m_element;
    public SlotElement SlotElement => m_element;
    
    public virtual void OccupySlot(SlotElementData data, SlotElement element)
    {
        m_slotElementData = data;
        m_element = element;
    }
    public override void EmptySlot()
    {
        m_slotElementData = null;
        m_element = null;
    }
    public override bool IsEmpty()
    {
        return m_element == null;
    }
    
}