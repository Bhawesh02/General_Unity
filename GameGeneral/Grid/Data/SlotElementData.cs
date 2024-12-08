using UnityEngine;

[CreateAssetMenu(fileName = "SlotElementData", menuName = "Grid/SlotElementData", order = 1)]
public class SlotElementData : ScriptableObject
{
    public Vector3 positionOnSlot;
    public SlotElement prefab;
}